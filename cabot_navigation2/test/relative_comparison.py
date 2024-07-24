#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2024  Miraikan - The National Museum of Emerging Science and Innovation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
###############################################################################

import os
import pandas as pd
import yaml
from argparse import ArgumentParser
from pathlib import Path
from typing import List


class ConsoleColor:
    GREEN = '\033[32m'
    RESET = '\033[0m'


def read_yaml_title(file_path: str) -> str:
    """Read the title from a YAML file."""
    try:
        with open(file_path, 'r') as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            return yaml_data.get('title', 'no title')
    except FileNotFoundError:
        print(f"Metadata file {file_path} not found.")
        return 'no title'


def read_csv_file(file_path: str) -> pd.DataFrame:
    """Read a CSV file into a DataFrame."""
    try:
        return pd.read_csv(file_path, index_col=0, encoding='utf-8')
    except FileNotFoundError:
        print(f"CSV file {file_path} not found.")
        return pd.DataFrame()


def read_test_summary_files(folder_paths: List[str]) -> List[pd.DataFrame]:
    """Read test summary files from the given folder paths."""
    data = []
    for folder_path in folder_paths:
        test_summary_path = os.path.join(folder_path, 'test_summary.csv')
        launch_metadata_path = os.path.join(folder_path, 'launch_metadata.yaml')

        title = read_yaml_title(launch_metadata_path)
        df = read_csv_file(test_summary_path)

        if not df.empty:
            df['Test run'] = Path(folder_path).name
            df['Test title'] = title
            df['Number of test cases'] = df['Number of success'] + df['Number of failure']
            data.append(df)

    return data


def read_test_evaluation_files(folder_paths: List[str]) -> List[pd.DataFrame]:
    """Read test evaluation files from the given folder paths."""
    data = []
    for folder_path in folder_paths:
        test_evaluation_path = os.path.join(folder_path, 'test_evaluation_results.csv')
        launch_metadata_path = os.path.join(folder_path, 'launch_metadata.yaml')

        title = read_yaml_title(launch_metadata_path)
        df = read_csv_file(test_evaluation_path)

        if not df.empty:
            df['Test run'] = Path(folder_path).name
            df['Test title'] = title
            data.append(df)

    return data


def summarize_by_module(data: List[pd.DataFrame], is_evaluation: bool = False) -> pd.DataFrame:
    """Summarize data by module."""
    combined_df = pd.concat(data)

    if is_evaluation:
        summary = combined_df.groupby(['Test module name', 'evaluator', 'Test run', 'Test title']).agg(
            **{'Number of test cases': ('value', 'count')},
            sum=('value', 'sum'),
            mean=('value', 'mean'),
            min=('value', 'min'),
            max=('value', 'max')
        )
    else:
        summary = combined_df.groupby(['Test module name', 'Test run', 'Test title']).agg({
            'Number of test cases': 'sum',
            'Number of success': 'sum',
            'Number of failure': 'sum'
        })
        summary['Success rate'] = summary['Number of success'] / summary['Number of test cases']

    return summary


def summarize_by_case(data: List[pd.DataFrame], is_evaluation: bool = False) -> pd.DataFrame:
    """Summarize data by case."""
    combined_df = pd.concat(data)

    if is_evaluation:
        summary = combined_df.groupby(['Test module name', 'Test case name', 'evaluator', 'Test run', 'Test title']).agg(
            value=('value', 'first')
        )
    else:
        summary = combined_df.groupby(['Test module name', 'Test case name', 'Test run', 'Test title']).agg({
            'Number of success': 'sum',
            'Number of failure': 'sum'
        })

    return summary


def print_summary(summary: pd.DataFrame, title: str) -> None:
    """Print the summary with a title."""
    print(f"{ConsoleColor.GREEN}{title}{ConsoleColor.RESET}")
    print(summary.to_string())
    print("")


def output_summary_to_csv(summary: pd.DataFrame, output_dir: str, filename: str) -> None:
    """Output the summary to a CSV file."""
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, filename)
    summary.to_csv(output_file)


def main() -> None:
    """Main function to parse arguments and process files."""
    parser = ArgumentParser()

    parser.add_argument('-f', '--folder_paths', action='append', nargs='*', type=str, help='Path to the folder containing test summary and test evaluation files')
    parser.add_argument('-o', '--output-dir', type=str, help='Directory where the relative comparison summary will be output')

    args = parser.parse_args()

    if not args.folder_paths:
        parser.error('Folder paths not given')

    folder_paths = [path for sublist in args.folder_paths for path in sublist]

    # Process test summary files
    summary_data = read_test_summary_files(folder_paths)
    if summary_data:
        test_summary_by_module = summarize_by_module(summary_data)
        test_summary_by_case = summarize_by_case(summary_data)
        print_summary(test_summary_by_module, "Test Summary by Test Module:")
        print_summary(test_summary_by_case, "Test Summary by Test Case:")
        if args.output_dir:
            output_summary_to_csv(test_summary_by_module, args.output_dir, 'test_summary_by_module_comparison.csv')
            output_summary_to_csv(test_summary_by_case, args.output_dir, 'test_summary_by_case_comparison.csv')

    # Process test evaluation files
    evaluation_data = read_test_evaluation_files(folder_paths)
    if evaluation_data:
        test_evaluation_by_module = summarize_by_module(evaluation_data, is_evaluation=True)
        test_evaluation_by_case = summarize_by_case(evaluation_data, is_evaluation=True)
        print_summary(test_evaluation_by_module, "Test Evaluation by Test Module:")
        print_summary(test_evaluation_by_case, "Test Evaluation by Test Case:")
        if args.output_dir:
            output_summary_to_csv(test_evaluation_by_module, args.output_dir, 'test_evaluation_by_module_comparison.csv')
            output_summary_to_csv(test_evaluation_by_case, args.output_dir, 'test_evaluation_by_case_comparison.csv')


if __name__ == "__main__":
    main()
