"""Object-relative search, original-pixel geometry and unchanged seed gates."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_semantic_search import PalletSemanticSearch, object_window, restore_geometry
from pallet_target_geometry import _leg_components


class Tensor:
    def __init__(self,data): self.data=np.asarray(data)
    def cpu(self): return self
    def numpy(self): return self.data


def result(boxes,scores,partial=False):
    masks=[]
    for box in boxes:
        x1,y1,x2,y2=np.rint(box).astype(int)
        mask=np.zeros((480,640),np.uint8)
        middle=y1+int((y2-y1)*.6);leg=max(8,int((x2-x1)*.16))
        mask[y1:middle,x1:x2]=1
        mask[middle:y2,x1:x1+leg]=1
        if not partial: mask[middle:y2,x2-leg:x2]=1
        masks.append(mask)
    return SimpleNamespace(
        boxes=SimpleNamespace(xyxy=Tensor(np.asarray(boxes).reshape(-1,4)),
                              cls=Tensor(np.ones(len(boxes))),conf=Tensor(scores)),
        masks=SimpleNamespace(data=Tensor(masks)) if masks else None,
        names={1:'emty pallet'})


class Model:
    def __init__(self,*responses): self.responses=list(responses);self.images=[];self.options=[]
    def predict(self,image,**kwargs):
        self.images.append(image.copy())
        self.options.append(kwargs)
        return [self.responses.pop(0)]


class SemanticSearchTest(unittest.TestCase):
    def setUp(self): self.image=np.full((480,640,3),100,np.uint8)

    def test_complete_seed_keeps_global_view_for_consecutive_fresh_confirmations(self):
        response=result([[110,260,250,350]],[.93])
        model=Model(response,response,response);search=PalletSemanticSearch()
        for i in range(3):
            found=search.predict(self.image+i,model)
            self.assertEqual(found.seed_index,0)
            self.assertEqual(found.search_window,(0,0,640,480))
            self.assertIsNone(search.window)
            np.testing.assert_array_equal(model.images[-1],self.image+i)

    def test_complete_low_confidence_target_uses_mild_context_crop_on_new_frames(self):
        # Logged scene: full frame 74.2%, old tight crop 71.9%, mild crop 88.5%.
        weak=result([[219,276,361,375]],[.7417])
        complete=result([[242,252,398,365]],[.8853])
        search=PalletSemanticSearch();model=Model(weak,complete,complete,complete)
        first=search.predict(self.image,model)
        self.assertIsNone(first.seed_index)
        self.assertIn('74.2%',first.status)
        window=search.window
        self.assertGreaterEqual(window[2]-window[0],640*.9)
        self.assertLess(window[2]-window[0],640)
        for i in range(1,4):
            found=search.predict(self.image+i,model)
            self.assertEqual(found.seed_index,0)
            self.assertEqual(found.search_window,window)
            self.assertEqual(search.window,window)
            self.assertTrue(np.all(model.images[-1]==100+i))

    def test_mild_crop_does_not_accept_weak_or_ambiguous_targets(self):
        weak=result([[219,276,361,375]],[.7417])
        for next_frame in (result([[242,252,398,365]],[.74]),
                           result([[50,100,200,220],[300,220,440,340]],[.93,.94])):
            search=PalletSemanticSearch();model=Model(weak,next_frame)
            self.assertIsNone(search.predict(self.image,model).seed_index)
            self.assertIsNone(search.predict(self.image+1,model).seed_index)
            self.assertIsNone(search.window)

    def test_failed_zoom_returns_to_global_instead_of_repeatedly_zooming_partial_mask(self):
        model=Model(result([[110,260,250,350]],[.53],partial=True),
                    result([[130,130,420,340]],[.93],partial=True),
                    result([[110,260,250,350]],[.93]))
        search=PalletSemanticSearch()
        self.assertIsNone(search.predict(self.image,model).seed_index)
        self.assertIsNotNone(search.window)
        self.assertIsNone(search.predict(self.image+1,model).seed_index)
        self.assertIsNone(search.window)
        final=search.predict(self.image+2,model)
        self.assertEqual(final.search_window,(0,0,640,480))
        self.assertEqual(final.seed_index,0)

    def test_successful_zoom_keeps_the_same_view_without_recursive_resizing(self):
        weak=result([[110,260,250,350]],[.53],partial=True)
        complete=result([[130,130,420,340]],[.93])
        model=Model(weak,complete,complete);search=PalletSemanticSearch()
        search.predict(self.image,model);window=search.window
        self.assertEqual(search.predict(self.image+1,model).seed_index,0)
        self.assertEqual(search.window,window)
        self.assertEqual(search.predict(self.image+2,model).search_window,window)

    def test_weak_partial_proposal_is_not_a_seed_but_next_frame_can_refine(self):
        model=Model(result([[340,260,410,340]],[.53],partial=True),
                    result([[180,130,420,290]],[.94]))
        search=PalletSemanticSearch()
        first=search.predict(self.image,model)
        self.assertIsNone(first.seed_index)
        self.assertNotEqual(search.window,(0,0,640,480))
        second=search.predict(self.image+1,model)
        self.assertEqual(second.seed_index,0)
        self.assertEqual(second.masks[0].shape,(480,640))
        self.assertEqual(len(_leg_components(second.masks[0],second.boxes[0])),2)
        self.assertTrue(np.all(model.images[1]==101))

    def test_full_search_accepts_complete_targets_at_different_positions_and_sizes(self):
        for box in ([20,100,140,180],[260,250,400,340],[470,40,620,150]):
            with self.subTest(box=box):
                search=PalletSemanticSearch()
                found=search.predict(self.image,Model(result([box],[.9])))
                self.assertEqual(found.seed_index,0)
                np.testing.assert_array_equal(found.boxes[0],box)

    def test_zoom_geometry_maps_back_to_original_pixels_at_different_locations(self):
        original=result([[220,140,420,300]],[.9])
        for window in ((0,0,320,240),(160,180,480,420),(320,0,640,240)):
            boxes,masks=restore_geometry(original.boxes.xyxy.numpy(),
                                        original.masks.data.numpy(),window,(480,640))
            x,y,_,_=window
            np.testing.assert_array_equal(boxes[0],[x+110,y+70,x+210,y+150])
            self.assertEqual(len(_leg_components(masks[0],boxes[0])),2)

    def test_clipped_missing_legs_and_low_confidence_cannot_qualify(self):
        for response in (result([[0,100,120,180]],[.99]),
                         result([[240,180,400,300]],[.99],partial=True),
                         result([[240,180,400,300]],[.74])):
            found=PalletSemanticSearch().predict(self.image,Model(response))
            self.assertIsNone(found.seed_index)

    def test_two_targets_remain_ambiguous_instead_of_selecting_strongest(self):
        model=Model(result([[30,100,150,180],[280,220,420,320]],[.99,.91]))
        search=PalletSemanticSearch()
        found=search.predict(self.image,model)
        self.assertIsNone(found.seed_index)
        self.assertIsNone(search.window)

    def test_green_identity_is_independent_of_location_and_still_requires_yolo(self):
        for green_box,wood_box in (([30,100,150,180],[280,220,420,320]),
                                   ([450,100,570,180],[40,220,180,320])):
            scene=self.image.copy();x,y,x2,y2=green_box;scene[y:y2,x:x2]=[30,130,40]
            model=Model(result([green_box,wood_box],[.93,.99]))
            found=PalletSemanticSearch('green').predict(scene,model)
            self.assertEqual(found.seed_index,0)
        self.assertIsNone(PalletSemanticSearch('green').predict(
            scene,Model(result([],[]))).seed_index)

    def test_two_green_pallets_do_not_silently_choose_a_station(self):
        boxes=[[30,100,150,180],[280,220,420,320]]
        scene=self.image.copy()
        for x,y,x2,y2 in boxes:scene[y:y2,x:x2]=[30,130,40]
        found=PalletSemanticSearch('green').predict(scene,Model(result(boxes,[.99,.91])))
        self.assertIsNone(found.seed_index)
        self.assertIn('found 2',found.status)

    def test_wood_only_is_not_a_fallback_when_green_is_missing(self):
        found=PalletSemanticSearch('green').predict(self.image,
            Model(result([[280,220,420,320]],[.99])))
        self.assertIsNone(found.seed_index)
        self.assertIn('found 0',found.status)

    def test_target_leaving_zoom_returns_to_global_search_and_can_reappear_elsewhere(self):
        search=PalletSemanticSearch()
        model=Model(result([[240,180,400,300]],[.9]),result([],[]),
                    result([[30,50,150,130]],[.9]))
        self.assertEqual(search.predict(self.image,model).seed_index,0)
        self.assertIsNone(search.predict(self.image,model).seed_index)
        self.assertIsNone(search.window)
        found=search.predict(self.image,model)
        self.assertEqual(found.search_window,(0,0,640,480))
        self.assertEqual(found.seed_index,0)

    def test_windows_expand_partial_objects_and_stay_inside_image(self):
        for box in ([2,2,60,70],[580,410,638,478],[340,260,410,340]):
            x,y,x2,y2=object_window(box,(480,640))
            self.assertTrue(0<=x<x2<=640 and 0<=y<y2<=480)
            self.assertTrue(x<=box[0] and y<=box[1] and x2>=box[2] and y2>=box[3])

    def test_network_size_does_not_change_camera_pixels_or_seed_mask_size(self):
        model=Model(result([[240,180,400,300]],[.94]))
        model._pallet_input_shape=(288,384)
        found=PalletSemanticSearch().predict(self.image,model)
        self.assertEqual(model.options[0]['imgsz'],(288,384))
        self.assertEqual(model.images[0].shape,(480,640,3))
        self.assertEqual(found.masks[0].shape,(480,640))
        np.testing.assert_array_equal(found.boxes[0],[240,180,400,300])


if __name__=='__main__': unittest.main()
