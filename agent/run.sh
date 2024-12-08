#!/bin/bash

challenge="4"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:p:r:h:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    4)
        source venv/bin/activate
        pip install -r requirements.txt
        python3 mainRobC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
esac
