#!/bin/bash
docker buildx build --platform linux/arm64 --push -t romir/basketbuddy .

