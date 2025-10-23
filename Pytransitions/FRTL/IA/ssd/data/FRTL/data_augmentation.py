#!/usr/bin/env python3
# _*_ coding:utf-8 _*_

# Autor: Marcus + ChatGPT
# Descrição:
#   - Divide dataset VOC em treino/val/test
#   - Aplica augmentation no conjunto de treino
#   - Atualiza train.txt e trainval.txt com as imagens aumentadas
#   - Mantém compatibilidade Pascal VOC

import os
import random
import glob
import shutil
import copy
import cv2
import xml.etree.ElementTree as ET
import albumentations as A
from albumentations.augmentations.geometric.rotate import SafeRotate

# -------------------------------
# Configurações principais
# -------------------------------
xmlfilepath = 'jetson/python/training/detection/ssd/data/FRTL/Annotations'
imgfilepath = 'jetson/python/training/detection/ssd/data/FRTL/JPEGImages'
output_base = 'jetson/python/training/detection/ssd/data/FRTL/AugmentedDataset'
txtsavepath = os.path.join(output_base, 'ImageSets/Main')
os.makedirs(txtsavepath, exist_ok=True)

trainval_percent = 0.8
train_percent = 0.9

# -------------------------------
# Augmentations a aplicar no treino
# -------------------------------
bbox_params = A.BboxParams(format='pascal_voc', label_fields=['class_labels'], clip=True)

augmentations = [
    ("flip", A.Compose([A.HorizontalFlip(p=1.0)], bbox_params=bbox_params)),
    ("flip_v", A.Compose([A.VerticalFlip(p=1.0)], bbox_params=bbox_params)),
    ("rotate", A.Compose([SafeRotate(limit=20, p=1.0)], bbox_params=bbox_params)),
    ("brightness_contrast", A.Compose([A.RandomBrightnessContrast(0.3, 0.3, p=1.0)], bbox_params=bbox_params)),
    ("hue_sat", A.Compose([A.HueSaturationValue(20, 30, 20, p=1.0)], bbox_params=bbox_params)),
    ("blur", A.Compose([A.GaussianBlur(blur_limit=(3, 7), p=1.0)], bbox_params=bbox_params)),
    ("noise", A.Compose([A.GaussNoise(p=1.0)], bbox_params=bbox_params)),
    ("affine", A.Compose([A.Affine(scale=(0.9,1.1), translate_percent=(0.05,0.1), rotate=(-15,15), shear=(-10,10), p=1.0)], bbox_params=bbox_params)),
]

# -------------------------------
# Funções auxiliares
# -------------------------------
def parse_voc_xml(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    boxes, labels = [], []
    for obj in root.findall("object"):
        label = obj.find("name").text
        bbox = obj.find("bndbox")
        xmin = int(float(bbox.find("xmin").text))
        ymin = int(float(bbox.find("ymin").text))
        xmax = int(float(bbox.find("xmax").text))
        ymax = int(float(bbox.find("ymax").text))
        boxes.append([xmin, ymin, xmax, ymax])
        labels.append(label)
    return tree, boxes, labels

def update_voc_xml(tree, boxes, labels, new_img_name, new_size):
    root = tree.getroot()
    if root.find("filename") is None:
        ET.SubElement(root, "filename").text = new_img_name
    else:
        root.find("filename").text = new_img_name

    if root.find("size") is None:
        size = ET.SubElement(root, "size")
        ET.SubElement(size, "width").text = str(new_size[1])
        ET.SubElement(size, "height").text = str(new_size[0])
        ET.SubElement(size, "depth").text = str(new_size[2] if len(new_size)==3 else 3)
    else:
        size = root.find("size")
        size.find("width").text = str(new_size[1])
        size.find("height").text = str(new_size[0])
        if size.find("depth") is not None:
            size.find("depth").text = str(new_size[2] if len(new_size)==3 else 3)

    for obj, box in zip(root.findall("object"), boxes):
        bbox = obj.find("bndbox")
        bbox.find("xmin").text = str(max(0, int(box[0])))
        bbox.find("ymin").text = str(max(0, int(box[1])))
        bbox.find("xmax").text = str(max(0, int(box[2])))
        bbox.find("ymax").text = str(max(0, int(box[3])))

def augment_image(img, boxes, labels, transform):
    aug = transform(image=img, bboxes=boxes, class_labels=labels)
    aug_img = aug["image"]
    aug_boxes = aug["bboxes"]
    return aug_img, aug_boxes

# -------------------------------
# Etapa 1: Dividir dataset
# -------------------------------
total_xml = glob.glob(os.path.join(xmlfilepath, '*.xml'))
num = len(total_xml)
ids = list(range(num))
tv = int(num * trainval_percent)
tr = int(tv * train_percent)
trainval = random.sample(ids, tv)
train = random.sample(trainval, tr)

for subset in ["train", "val", "test"]:
    os.makedirs(os.path.join(output_base,'JPEGImages',subset), exist_ok=True)
    os.makedirs(os.path.join(output_base,'Annotations',subset), exist_ok=True)

ftrainval = open(os.path.join(txtsavepath,'trainval.txt'),'w')
ftest = open(os.path.join(txtsavepath,'test.txt'),'w')
ftrain = open(os.path.join(txtsavepath,'train.txt'),'w')
fval = open(os.path.join(txtsavepath,'val.txt'),'w')

for i in ids:
    name = os.path.basename(total_xml[i])[:-4]
    img_src = os.path.join(imgfilepath, name+".jpg")
    xml_src = os.path.join(xmlfilepath, name+".xml")

    if not os.path.exists(img_src):
        continue

    if i in trainval:
        ftrainval.write(name+'\n')
        if i in train:
            subset="train"
            ftrain.write(name+'\n')
        else:
            subset="val"
            fval.write(name+'\n')
    else:
        subset="test"
        ftest.write(name+'\n')

    shutil.copy(img_src, os.path.join(output_base,'JPEGImages',subset,name+".jpg"))
    shutil.copy(xml_src, os.path.join(output_base,'Annotations',subset,name+".xml"))

ftrainval.close(); ftrain.close(); fval.close(); ftest.close()
print("✅ Divisão concluída!")

# -------------------------------
# Etapa 2: Data Augmentation no treino
# -------------------------------
train_img_dir = os.path.join(output_base,'JPEGImages/train')
train_xml_dir = os.path.join(output_base,'Annotations/train')
output_img_dir = train_img_dir
output_xml_dir = train_xml_dir

train_files = [f for f in os.listdir(train_img_dir) if f.endswith(".jpg")]
print(f"🧩 Aplicando augmentation em {len(train_files)} imagens de treino...")

for file in train_files:
    img_path = os.path.join(train_img_dir, file)
    xml_path = os.path.join(train_xml_dir, file.replace(".jpg",".xml"))

    img = cv2.imread(img_path)
    if img is None:
        print(f"⚠️ Erro ao ler {file}")
        continue

    try:
        tree, boxes, labels = parse_voc_xml(xml_path)
    except Exception as e:
        print(f"⚠️ Erro ao ler XML {file}: {e}")
        continue

    for aug_name, transform in augmentations:
        try:
            boxes_copy = copy.deepcopy(boxes)
            labels_copy = copy.deepcopy(labels)

            aug_img, aug_boxes = augment_image(img, boxes_copy, labels_copy, transform)
            if len(aug_boxes) == 0:
                continue

            new_name = file.replace(".jpg", f"_{aug_name}.jpg")
            new_xml = file.replace(".jpg", f"_{aug_name}.xml")

            new_tree = copy.deepcopy(tree)
            update_voc_xml(new_tree, aug_boxes, labels_copy, new_name, aug_img.shape)

            cv2.imwrite(os.path.join(output_img_dir,new_name), aug_img)
            new_tree.write(os.path.join(output_xml_dir,new_xml))

            # Adiciona imagem aumentada aos arquivos train.txt e trainval.txt
            new_basename = new_name.replace(".jpg","")
            with open(os.path.join(txtsavepath,'train.txt'),'a') as ftrain:
                ftrain.write(new_basename+'\n')
            with open(os.path.join(txtsavepath,'trainval.txt'),'a') as ftrainval:
                ftrainval.write(new_basename+'\n')

        except Exception as e:
            print(f"⚠️ Erro com {file} - {aug_name}: {e}")
            continue

print("✅ Augmentation finalizado com sucesso!")
