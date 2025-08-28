#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright (c) 2025  IBM Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import torch
from torchvision import transforms


def get_image_feature_extractor(feature="cosplace"):
    return ImageFeatureExtractor.get_instance(feature)


class ImageFeatureExtractor:
    _reusable_instances = {}

    @classmethod
    def get_instance(cls, feature="cosplace"):
        if feature not in cls._reusable_instances.keys():
            cls._reusable_instances[feature] = cls(feature)
        return cls._reusable_instances.get(feature)

    def __init__(self, feature="cosplace"):
        if torch.cuda.is_available():
            device_str = "cuda"
        elif torch.backends.mps.is_available():
            device_str = "mps"
        else:
            device_str = "cpu"
        self.device_str = device_str
        self.device = torch.device(device_str)

        if feature == "cosplace":
            model = torch.hub.load("gmberton/cosplace", "get_trained_model",
                                   backbone="ResNet50", fc_output_dim=2048)
            self.model = model.to(self.device)
            self.transformer = transforms.Compose([
                                            transforms.Resize(480),
                                            transforms.ToTensor(),
                                            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                                 std=[0.229, 0.224, 0.225]),
                                            ])
        elif feature == "megaloc":
            model = torch.hub.load("gmberton/MegaLoc", "get_trained_model")
            self.model = model.to(self.device)
            self.transformer = transforms.Compose([
                                            transforms.Resize(480),
                                            transforms.ToTensor(),
                                            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                                 std=[0.229, 0.224, 0.225]),
                                            ])
        else:
            raise RuntimeError(F"Invalid {feature = }")

    def extract_feature(self, images=None):
        tensors = []
        for img_raw in images:
            tensors.append(self.transformer(img_raw))
        stack = torch.stack(tensors, dim=0)
        stack = stack.to(self.device)

        with torch.no_grad():
            des = self.model(stack)
        des = des.cpu().numpy()
        return des