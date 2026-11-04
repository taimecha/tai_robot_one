"""Global YOLO proposals, then object-relative refinement on NEW RGB.

Weak proposals only schedule another view. Seeds still need the original
confidence gate and two unclipped front legs, in original camera pixels.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from pallet_detection_selection import unique_pallet_indices
from pallet_target_geometry import _leg_components

MIN_SEMANTIC_CONFIDENCE = .75


@dataclass
class SemanticResult:
    boxes: np.ndarray
    classes: np.ndarray
    confidence: np.ndarray
    masks: list
    names: dict
    pallet_indices: list
    seed_index: object
    status: str
    search_window: tuple


def object_window(bbox, shape, minimum_fraction=0.):
    """Grow a partial proposal in either direction, preserving image aspect."""
    height, width = shape
    x1, y1, x2, y2 = map(float, bbox)
    if not all(math.isfinite(v) for v in bbox) or x2 <= x1 or y2 <= y1:
        raise ValueError('Invalid semantic proposal box')
    view_width = min(float(width), max(width*minimum_fraction,2.5*(x2-x1), 3.0*(y2-y1)*width/height,
                                     width/4.0))
    view_height = view_width * height / width
    left = min(max(0., (x1+x2-view_width)/2), width-view_width)
    top = min(max(0., (y1+y2-view_height)/2), height-view_height)
    return (int(math.floor(left)), int(math.floor(top)),
            min(width,int(math.ceil(left+view_width))),
            min(height,int(math.ceil(top+view_height))))


def restore_geometry(boxes, masks, window, shape):
    height, width = shape
    x1,y1,x2,y2 = window
    scale = np.array([(x2-x1)/width,(y2-y1)/height]*2)
    restored = np.asarray(boxes,dtype=float).reshape(-1,4)*scale+[x1,y1,x1,y1]
    full_masks = []
    for mask in masks:
        full = np.zeros(shape,np.uint8)
        full[y1:y2,x1:x2] = cv2.resize(
            (np.asarray(mask)>.5).astype(np.uint8),(x2-x1,y2-y1),
            interpolation=cv2.INTER_NEAREST)
        full_masks.append(full.astype(bool))
    return restored,full_masks


def matches_color(image, mask, target_color):
    if target_color == 'any':
        return True
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    green = ((hsv[:,:,0]>=32)&(hsv[:,:,0]<=100)&
             (hsv[:,:,1]>=45)&(hsv[:,:,2]>=35))
    return bool(mask.any() and np.mean(green[mask])>=.35)


class PalletSemanticSearch:
    def __init__(self,target_color='any'):
        if target_color not in ('any','green'):
            raise ValueError('Unknown pallet target color')
        self.target_color = target_color
        self.reset()

    def reset(self):
        self.window = None

    def predict(self,image,model,confidence=.60,iou=.30,device='cpu'):
        shape = image.shape[:2]
        height,width = shape
        window = self.window or (0,0,width,height)
        x1,y1,x2,y2 = window
        view = cv2.resize(image[y1:y2,x1:x2],(width,height))
        result = model.predict(view,conf=min(confidence,.20),iou=iou,
                               imgsz=getattr(model,'_pallet_input_shape',shape),
                               retina_masks=True,device=device,verbose=False)[0]
        source_boxes = result.boxes.xyxy.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        source_masks = result.masks.data.cpu().numpy() if result.masks is not None else []
        boxes,masks = restore_geometry(source_boxes,source_masks,window,shape)
        indices = unique_pallet_indices(boxes,classes,scores,result.names)
        candidates = [i for i in indices if i<len(masks) and
                      matches_color(image,masks[i],self.target_color)]
        seed_index = None
        status = f'expected one target pallet, found {len(candidates)}'
        if len(candidates)==1:
            index = candidates[0]
            source = source_boxes[index]
            clipped = (source[0]<2 or source[1]<2 or
                       source[2]>width-2 or source[3]>height-2)
            complete_geometry=False
            try:
                if clipped:
                    raise ValueError('pallet clipped by recognition view')
                _leg_components(masks[index],boxes[index])
                complete_geometry=True
                if scores[index]<max(MIN_SEMANTIC_CONFIDENCE,confidence):
                    raise ValueError('semantic confidence below acquisition gate '
                        f'({scores[index]*100:.1f}% < {max(MIN_SEMANTIC_CONFIDENCE,confidence)*100:.1f}%)')
                seed_index = index
                status = 'YOLO_SEED'
            except ValueError as exc:
                status = 'REFINING: '+str(exc)
            # Keep a successful view fixed for consecutive confirmations.
            # Re-zooming an already complete seed changed its segmentation
            # and could merge the legs on every subsequent inference.
            # A failed zoom returns to the full image instead of recursively
            # magnifying a partial/incorrect mask. Always use a NEW frame.
            if seed_index is not None:
                self.window = None if window==(0,0,width,height) else window
            elif clipped or window!=(0,0,width,height):
                self.window = None
            else:
                # Both legs are already complete: preserve scene context with
                # a mild zoom on the NEXT fresh frame. The tighter partial-leg
                # crop can lower confidence and oscillate between class labels.
                self.window = object_window(boxes[index],shape,
                    minimum_fraction=.90 if complete_geometry else 0.)
        else:
            # Multiple eligible targets stay ambiguous, even after a zoom.
            self.window = None
        return SemanticResult(boxes,classes,scores,masks,result.names,indices,
                              seed_index,status,window)
