import os
os.environ["PYTHONPATH"] = "/home/aglover/projects/bimvee"

from bimvee.importIitYarp import importIitYarp
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--data',  dest='data', type=str, required=False, help='Path to input data')
args = parser.parse_args("")

## import data
if args.data is None:
    print("Using default path: /home/aglover/HPE/data/fault_button/s1_v1")
    args.data = "/home/aglover/HPE/data/fault_button/s1_v1"

try:
    events = importIitYarp(filePathOrName=args.data)
except:
    print("could not import data")
    exit

## Processing
print(events['ts'][0])

