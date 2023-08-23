#!/bin/bash
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created by  : Alexandre BERNE
# Creation date: 26/07/2023
# version ='1.0'
# ---------------------------------------------------------------------------
# Bash script to manage the prod_cons project traces
# ---------------------------------------------------------------------------

TRACE_FOLDER="/tmp/prod_cons"
PROJECT_TRACE_FILE="prod_cons_trace.json"

# trap ctrl-c and call ctrl_c()
function ctrl_c()
{
  sleep 1
  echo "{\"traceEvents\": [" > $PROJECT_TRACE_FILE
  
  for trace_file in "$TRACE_FOLDER"/*
  do
    echo "Adding ${trace_file}"
    # add trace_file, except first and last lines
    sed '1,1d; $d' ${trace_file} >> $PROJECT_TRACE_FILE
    echo "," >> $PROJECT_TRACE_FILE
  done
  
  # because we added a comma, we need to add a dummy structure to make the JSON syntactically correct
  echo "{\"name\": \"dummy\"}" >> $PROJECT_TRACE_FILE
  echo "]}" >> $PROJECT_TRACE_FILE
  
  # removing non merged traces
  rm -rf $TRACE_FOLDER

  echo "Execution of prod_cons completed !"
  echo "prod_cons trace is located at:"
  pwd
  exit 0
}

# Check if traces destination folder exists
if [ -d $TRACE_FOLDER ] 
then
  rm -rf $TRACE_FOLDER
fi

# Create traces folder to store new traces
mkdir $TRACE_FOLDER

# Catch CTRL+c
trap ctrl_c SIGINT

while :
do
  sleep 1
done