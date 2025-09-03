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

import base64
import json
import logging
import numpy as np
from io import BytesIO
from PIL import Image
from fastapi import APIRouter, Request, Response
from fastapi import HTTPException

from ..vpr.image_feature_extractor import get_image_feature_extractor

router = APIRouter()

logger = logging.getLogger(__name__)


@router.post("/load_model")
async def load_model(feature: str = "cosplace"):
    try:
        _ = get_image_feature_extractor(feature)
    except Exception:
        logger.exception("Failed get_image_feature_extractor in load_model")
        raise HTTPException(status_code=500, detail="Failed to get image feature extractor model")


@router.post('/extract_feature')
async def extract_feature(request: Request,
                          feature: str = "cosplace",
                          format: str = "npy"
                          ):
    logger.info("extract_feature post")

    try:
        images = await request.json()
    except json.decoder.JSONDecodeError:
        response = {
            "error": "JSON decoding error."
        }
        return response

    # convert base64 encoded images to PIL images
    pil_images = []
    for image in images:
        image_uri = image["image_uri"]
        pil_image = Image.open(BytesIO(base64.b64decode(image_uri.split(",")[1])))
        pil_images.append(pil_image)

    # extract feature
    try:
        model = get_image_feature_extractor(feature)
    except Exception:
        logger.exception("Failed get_image_feature_extractor in extract_feature")
        raise HTTPException(status_code=500, detail="Failed to get image feature extractor model")
    descriptors = model.extract_feature(images=pil_images)

    if format == "npy":
        buffer = BytesIO()
        np.save(buffer, descriptors, allow_pickle=False)
        data = buffer.getvalue()
        headers = {
            "Content-Disposition": 'attachment; filename="array.npy"',
            "Cache-Control": "no-store",
        }
        response = Response(content=data, media_type="application/octet-stream", headers=headers)
    elif format == "json":
        response = []
        for desc in descriptors:
            response.append(str(desc))
    else:
        response = {
            "error": F"unknown format ({format})"
        }

    return response
