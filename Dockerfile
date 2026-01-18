FROM python:3.11-slim

# Install Pandas for GNSS-Logger parsing.
RUN pip install --no-cache-dir pandas

# Install tqdm to see the progress of the program execution.
RUN  pip install --no-cache-dir tqdm 

# Install tkinter for GUI matplotlib plots.
RUN apt-get update && apt-get install x11-utils python3-tk tk -y

# Install matplotlib for plotting the dataframe columns hist.
RUN pip install --no-cache-dir matplotlib scipy

RUN pip install numpy==2.0.0 simplekml

WORKDIR /code
COPY . .
WORKDIR /code/src