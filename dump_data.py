from utils import dump_data_to_csv
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("input_log")
parser.add_argument("output_dir")
parser.add_argument("--time_limit", type=float, default=None)
args = parser.parse_args()

dump_data_to_csv(args.input_log, args.output_dir, args.time_limit)