FROM ubuntu:16.04

RUN apt update && apt -y install python
RUN apt update && apt -y install python-pip
RUN pip install pika==1.1.0

COPY *.py /
COPY base/ /base/
COPY tester_team/ /tester_team/
