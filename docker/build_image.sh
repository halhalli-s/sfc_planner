#!/bin/bash
echo "Building SFC Planner Docker Image..."
docker build -t prance:sfc-planner .
echo "Build complete!"
docker images | grep prance