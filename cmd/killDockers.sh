#!/bin/bash
docker ps --filter name=testware -aq | xargs docker kill