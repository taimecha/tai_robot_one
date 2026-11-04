#!/usr/bin/env python3
"""Build a smaller-input NCNN model from the existing segmentation weights."""
import argparse
from pathlib import Path
import shutil
from datetime import datetime
import os


def activate_model(project,output):
    """Keep the established launch path, with a recoverable previous model."""
    stable=project/'data/runtime_models/best_v3_ncnn_model'
    if stable.is_dir() and not stable.is_symlink():
        assert (stable/'metadata.yaml').is_file()
        backup=stable.with_name(stable.name+'_backup_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
        stable.rename(backup)
    temporary=stable.with_name(stable.name+'.new')
    if temporary.is_symlink(): temporary.unlink()
    temporary.symlink_to(os.path.relpath(output,stable.parent))
    temporary.replace(stable)
    return stable


def main():
    project=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights',type=Path,default=project/'models/best_v3.pt')
    parser.add_argument('--height',type=int,default=288)
    parser.add_argument('--width',type=int,default=384)
    parser.add_argument('--activate',action='store_true',
                        help='Back up the current runtime model and update the established launch path')
    args=parser.parse_args()
    if not args.weights.is_file() or min(args.height,args.width)<128 or args.height%32 or args.width%32:
        parser.error('Existing weights and input dimensions divisible by 32 are required')
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(1)
    destination=project/'data/runtime_models'/f'{args.weights.stem}_{args.width}'
    destination.mkdir(parents=True,exist_ok=True)
    copied=destination/args.weights.name
    shutil.copy2(args.weights,copied)
    model=YOLO(str(copied),task='segment')
    output=model.export(format='ncnn',imgsz=(args.height,args.width),device='cpu',
                        batch=1,quantize=32)
    print('NCNN model:',output,flush=True)
    if args.activate:
        print('Active runtime path:',activate_model(project,Path(output)),flush=True)


if __name__=='__main__': main()
