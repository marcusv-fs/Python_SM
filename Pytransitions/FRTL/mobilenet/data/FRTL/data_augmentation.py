#!/usr/bin/env python3
# _*_ coding:utf-8 _*_

import os
import random
import glob
import shutil
import copy
import cv2
import warnings
import xml.etree.ElementTree as ET
import albumentations as A
from albumentations.augmentations.geometric.rotate import SafeRotate

# Suprimir avisos inofensivos do Albumentations sobre BBox em filtros de cor
warnings.filterwarnings("ignore", category=UserWarning, module="albumentations")

# -------------------------------
# Configurações principais
# -------------------------------
xmlfilepath = '/home/marcusv-fs/Programacao/Python_SM/Pytransitions/FRTL/mobilenet/data/FRTL/Annotations'
imgfilepath = '/home/marcusv-fs/Programacao/Python_SM/Pytransitions/FRTL/mobilenet/data/FRTL/JPEGImages'
output_base = '/home/marcusv-fs/Programacao/Python_SM/Pytransitions/FRTL/mobilenet/data/FRTL/AugmentedDataset'

# Pastas de saída (Padrão VOC)
out_xml_dir = os.path.join(output_base, 'Annotations')
out_img_dir = os.path.join(output_base, 'JPEGImages')
txtsavepath = os.path.join(output_base, 'ImageSets/Main')

os.makedirs(out_xml_dir, exist_ok=True)
os.makedirs(out_img_dir, exist_ok=True)
os.makedirs(txtsavepath, exist_ok=True)

trainval_percent = 0.8
train_percent = 0.9

# -------------------------------
# Augmentations
# -------------------------------
bbox_params = A.BboxParams(format='pascal_voc', label_fields=['class_labels'], clip=True)

augmentations = [
    ("flip", A.Compose([A.HorizontalFlip(p=1.0)], bbox_params=bbox_params)),
    ("rotate", A.Compose([SafeRotate(limit=20, p=1.0)], bbox_params=bbox_params)),
    ("brightness", A.Compose([A.RandomBrightnessContrast(0.3, 0.3, p=1.0)], bbox_params=bbox_params)),
    ("blur", A.Compose([A.GaussianBlur(blur_limit=(3, 7), p=1.0)], bbox_params=bbox_params)),
    ("noise", A.Compose([A.GaussNoise(p=1.0)], bbox_params=bbox_params)),
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
    root.find("filename").text = new_img_name
    size = root.find("size")
    size.find("width").text = str(new_size[1])
    size.find("height").text = str(new_size[0])
    
    for obj, box in zip(root.findall("object"), boxes):
        bbox = obj.find("bndbox")
        bbox.find("xmin").text = str(max(0, int(box[0])))
        bbox.find("ymin").text = str(max(0, int(box[1])))
        bbox.find("xmax").text = str(max(0, int(box[2])))
        bbox.find("ymax").text = str(max(0, int(box[3])))

# -------------------------------
# Etapa 1: Dividir dataset e copiar arquivos
# -------------------------------
total_xml = glob.glob(os.path.join(xmlfilepath, '*.xml'))
num = len(total_xml)
ids = list(range(num))
tv = int(num * trainval_percent)
tr = int(tv * train_percent)
trainval_idx = random.sample(ids, tv)
train_idx = random.sample(trainval_idx, tr)

# Listas para guardar os nomes das imagens de treino para a Etapa 2
train_names_list = []

with open(os.path.join(txtsavepath,'trainval.txt'),'w') as ftrainval, \
     open(os.path.join(txtsavepath,'test.txt'),'w') as ftest, \
     open(os.path.join(txtsavepath,'train.txt'),'w') as ftrain, \
     open(os.path.join(txtsavepath,'val.txt'),'w') as fval:

    for i in ids:
        name = os.path.basename(total_xml[i])[:-4]
        img_src = os.path.join(imgfilepath, name+".jpg")
        xml_src = os.path.join(xmlfilepath, name+".xml")

        if not os.path.exists(img_src):
            continue

        # Copia o arquivo original para o novo dataset
        shutil.copy(img_src, os.path.join(out_img_dir, name+".jpg"))
        shutil.copy(xml_src, os.path.join(out_xml_dir, name+".xml"))

        if i in trainval_idx:
            ftrainval.write(name+'\n')
            if i in train_idx:
                ftrain.write(name+'\n')
                train_names_list.append(name) # Salva para augmentation
            else:
                fval.write(name+'\n')
        else:
            ftest.write(name+'\n')

print(f"✅ Divisão concluída! {len(train_names_list)} imagens na lista de treino.")

# -------------------------------
# Etapa 2: Data Augmentation (Apenas no que foi definido como treino)
# -------------------------------
print(f"🧩 Aplicando augmentation...")

for name in train_names_list:
    img_path = os.path.join(out_img_dir, name + ".jpg")
    xml_path = os.path.join(out_xml_dir, name + ".xml")

    img = cv2.imread(img_path)
    if img is None: continue

    try:
        tree, boxes, labels = parse_voc_xml(xml_path)
    except: continue

    for aug_name, transform in augmentations:
        try:
            aug = transform(image=img, bboxes=boxes, class_labels=labels)
            aug_img, aug_boxes = aug["image"], aug["bboxes"]

            if len(aug_boxes) == 0: continue

            new_name_base = f"{name}_{aug_name}"
            
            # Salva Imagem e XML aumentados
            cv2.imwrite(os.path.join(out_img_dir, f"{new_name_base}.jpg"), aug_img)
            
            new_tree = copy.deepcopy(tree)
            update_voc_xml(new_tree, aug_boxes, labels, f"{new_name_base}.jpg", aug_img.shape)
            new_tree.write(os.path.join(out_xml_dir, f"{new_name_base}.xml"))

            # Atualiza os arquivos .txt com a nova imagem aumentada
            with open(os.path.join(txtsavepath,'train.txt'),'a') as f:
                f.write(new_name_base + '\n')
            with open(os.path.join(txtsavepath,'trainval.txt'),'a') as f:
                f.write(new_name_base + '\n')

        except Exception as e:
            print(f"⚠️ Erro em {name} ({aug_name}): {e}")

print("✅ Processo finalizado com sucesso!")