#!/bin/bash
set -e

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

set -x


sudo apt install -y \
	python-frozendict \
	libxslt-dev \
	libxml2-dev \
	python-lxml \
	python-bs4 \
	python-tables \
    python-sklearn \
    apt-file \
    iftop \
    atop \
    ntpdate \
    python-termcolor \
    python-sklearn \
    libatlas-base-dev \
    python-dev \
    ipython \
    python-sklearn \
    python-smbus

sudo apt remove -y \
	python-ruamel.yaml \
	python-ruamel.ordereddict

# These don't have an APT package

pip install --upgrade --user \
	PyContracts==1.7.15 \
    DecentLogs==1.1.2\
	QuickApp==1.3.8 \
	conftools==1.9.1 \
	comptests==1.4.10 \
	procgraph==1.10.6 \
	pymongo==3.5.1 \
	ruamel.yaml==0.15.34




# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel
