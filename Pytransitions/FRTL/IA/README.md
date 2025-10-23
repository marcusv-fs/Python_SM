# pytorch-detection
Training of ONNX-compatible object detection networks with PyTorch.  Current support includes:

* [PyTorch-SSD](https://github.com/dusty-nv/pytorch-ssd) (SSD-Mobilenet)

Como treinar
python3 train_ssd.py --dataset-type=voc --data=data/FRTL --model-dir=models/FRTL --epochs 120 --pretrained-ssd="models/mobilenet-v1-ssd-mp-0_675_1.pth"

Para avaliar
python eval_ssd.py --net mb1-ssd --model models/FRTL/[REDE] --dataset data/FRTL

Após, exporte para onnx
python onnx_export.py   --net mb1-ssd   --model-dir models/FRTL   --labels models/FRTL/labels.txt   --width 300   --height 300

