#!/bin/bash
rm workspace.tar.gz
rm -r workspace/devel
rm -r workspace/build
tar -czvf workspace.tar.gz workspace
scp workspace.tar.gz robot@192.168.8.191:workspace.tar.gz
rm workspace.tar.gz
