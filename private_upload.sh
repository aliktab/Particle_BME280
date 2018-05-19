#!/usr/bin/env bash

cd $(dirname "$0")

# we have to make separate folder and copy lib files there because of
# the stupid particle tool which fails to upload library if it contains PDFs

mkdir _particle_upload

cp -R src             _particle_upload
cp -R examples        _particle_upload
cp library.properties _particle_upload
cp    LICENSE         _particle_upload
cp    README.md       _particle_upload

cd _particle_upload

particle library upload

cd ..
rm -R _particle_upload
