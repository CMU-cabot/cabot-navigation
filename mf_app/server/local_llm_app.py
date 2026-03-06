#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from fastapi import FastAPI

from .routers import local_llm

app = FastAPI()


@app.get("/")
def read_root():
    return {"message": "local llm server"}


@app.on_event("startup")
def startup_local_llm():
    local_llm.engine.initialize_on_startup()


app.include_router(local_llm.router)
