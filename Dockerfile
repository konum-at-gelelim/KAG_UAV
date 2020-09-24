FROM ubuntu:16.04

RUN apt-get update && apt -y install python
RUN apt-get update && apt -y install python-pip
RUN python2.7 -mpip install --upgrade pip
RUN python2.7 -mpip install pika==1.1.0
RUN python2.7 -mpip install numpy scipy matplotlib

COPY *.py /
COPY base/ /base/
