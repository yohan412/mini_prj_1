#!/usr/bin/env python3
"""
PC에서 실행: 준비된 YOLO 데이터셋(data.yaml)으로 YOLOv8 모델을 학습합니다.

사전 준비 (직접 구성):
  dataset/
    data.yaml
    train/images/
    train/labels/
    val/images/
    val/labels/

실행 예:
  python3 train_yolo.py --data dataset/data.yaml
"""

import argparse
from pathlib import Path

from ultralytics import YOLO

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BASE_MODEL = SCRIPT_DIR / 'yolov8n.pt'
DEFAULT_DATA_YAML = SCRIPT_DIR / 'dataset' / 'data.yaml'
DEFAULT_PROJECT = SCRIPT_DIR / 'runs' / 'train'
DEFAULT_NAME = 'pinky_fruit'


def parse_args():
  parser = argparse.ArgumentParser(description='Train YOLOv8 with a prepared dataset')
  parser.add_argument('--data', default=str(DEFAULT_DATA_YAML), help='Path to data.yaml')
  parser.add_argument('--model', default=str(DEFAULT_BASE_MODEL), help='Base model (.pt)')
  parser.add_argument('--epochs', type=int, default=100)
  parser.add_argument('--imgsz', type=int, default=640)
  parser.add_argument('--batch', type=int, default=16)
  parser.add_argument('--project', default=str(DEFAULT_PROJECT), help='Training output directory')
  parser.add_argument('--name', default=DEFAULT_NAME, help='Run name under project/')
  parser.add_argument('--device', default='', help='cuda device id or cpu (default: auto)')
  parser.add_argument('--patience', type=int, default=20, help='Early stopping patience')
  return parser.parse_args()


def main():
  args = parse_args()

  data_path = Path(args.data)
  model_path = Path(args.model)

  if not data_path.is_file():
    raise FileNotFoundError(
      f'data.yaml not found: {data_path}\n'
      'dataset/data.yaml 을 직접 준비한 뒤 --data 로 경로를 지정하세요.'
    )
  if not model_path.is_file():
    raise FileNotFoundError(f'Base model not found: {model_path}')

  model = YOLO(str(model_path))

  print(f'Data: {data_path}')
  print(f'Base model: {model_path}')
  print(f'Epochs: {args.epochs}, imgsz: {args.imgsz}, batch: {args.batch}')

  results = model.train(
    data=str(data_path),
    epochs=args.epochs,
    imgsz=args.imgsz,
    batch=args.batch,
    project=args.project,
    name=args.name,
    device=args.device or None,
    patience=args.patience,
    exist_ok=True,
  )

  best_weights = Path(results.save_dir) / 'weights' / 'best.pt'
  print(f'\nTraining finished.')
  print(f'Best weights: {best_weights}')
  print(f'\nDetection test:')
  print(f'  python3 yolo_stream_viewer.py --model {best_weights}')


if __name__ == '__main__':
  main()
