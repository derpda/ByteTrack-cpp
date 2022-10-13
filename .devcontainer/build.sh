#!/bin/bash
set -ex

VERSION=$(cat VERSION)
docker build . -t byte_track_dev:$VERSION
