#!/usr/bin/env python3

###############################################################################
# Copyright (c) 2024  Kufusha Inc.
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
from optparse import OptionParser
from typing import List


class Color:
    GREEN = '\033[32m'
    RESET = '\033[0m'


def read_test_summary_files(folder_paths: List[str]) -> List[pd.DataFrame]:
    data = []
    for folder_path in folder_paths:
        test_summary_path = os.path.join(folder_path, 'test_summary.csv')
        launch_metadata_path = os.path.join(folder_path, 'launch_metadata.yaml')

        try:
            title = ""
            with open(launch_metadata_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)
                title = yaml_data.get('title', 'no title')

            df = pd.read_csv(test_summary_path, index_col=0, encoding='utf-8')
            df['Test run'] = os.path.basename(folder_path)
            df['Test title'] = title
            df['Number of test cases'] = df['Number of success'] + df['Number of failure']
            data.append(df)
        except FileNotFoundError:
            print(f"File {test_summary_path} not found.")
    return data


def read_test_evaluation_files(folder_paths: List[str]) -> List[pd.DataFrame]:
    data = []
    for folder_path in folder_paths:
        test_evaluation_path = os.path.join(folder_path, 'test_evaluation_results.csv')
        launch_metadata_path = os.path.join(folder_path, 'launch_metadata.yaml')

        try:
            title = ""
            with open(launch_metadata_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)
                title = yaml_data.get('title', 'no title')

            df = pd.read_csv(test_evaluation_path, index_col=0, encoding='utf-8')
            df['Test run'] = os.path.basename(folder_path)
            df['Test title'] = title
            data.append(df)
        except FileNotFoundError:
            print(f"File {test_evaluation_path} not found.")
    return data


def summarize_test_summary_by_module(data: List[pd.DataFrame]) -> pd.DataFrame:
    combined_df = pd.concat(data)
    summary = combined_df.groupby(['Test module name', 'Test run', 'Test title']).agg({
        'Number of test cases': 'sum',
        'Number of success': 'sum',
        'Number of failure': 'sum'
    })
    summary['Success rate'] = summary['Number of success'] / (summary['Number of success'] + summary['Number of failure'])
    return summary


def summarize_test_summary_by_case(data: List[pd.DataFrame]) -> pd.DataFrame:
    combined_df = pd.concat(data)
    summary = combined_df.groupby(['Test module name', 'Test case name', 'Test run', 'Test title']).agg({
        'Number of success': 'sum',
        'Number of failure': 'sum'
    })
    return summary


def summarize_test_evaluation_by_module(data: List[pd.DataFrame]) -> pd.DataFrame:
    combined_df = pd.concat(data)
    summary = combined_df.groupby(['Test module name', 'evaluator', 'Test run', 'Test title']).agg(
        **{'Number of test cases': ('value', 'count')},
        sum=('value', 'sum'),
        mean=('value', 'mean'),
        min=('value', 'min'),
        max=('value', 'max')
    )
    return summary


def summarize_test_evaluation_by_case(data: List[pd.DataFrame]) -> pd.DataFrame:
    combined_df = pd.concat(data)
    summary = combined_df.groupby(['Test module name', 'Test case name', 'evaluator', 'Test run', 'Test title']).agg(
        value=('value', 'first')
    )
    return summary


def print_test_summary(summary: pd.DataFrame, by_module: bool = True) -> None:
    if by_module:
        print(f"{Color.GREEN}Test Summary by Test Module: {Color.RESET}")
    else:
        print(f"{Color.GREEN}Test summary by Test Case: {Color.RESET}")
    print(summary.to_string())
    print("")


def print_test_evaluation(summary: pd.DataFrame, by_module: bool = True) -> None:
    if by_module:
        print(f"{Color.GREEN}Test Evaluation by Test Module: {Color.RESET}")
    else:
        print(f"{Color.GREEN}Test Evaluation by Test Case: {Color.RESET}")
    print(summary.to_string())
    print("")


def output_summary_to_csv(summary: pd.DataFrame, output_dir: str, filename: str) -> None:
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.join(output_dir, filename)
    summary.to_csv(output_file)


def main():
    parser = OptionParser()

    parser.add_option('-f', '--folder_paths', action='append', type=str, help='Path to the folder containing test summary and test evaluation files')
    parser.add_option('-o', '--output-dir', type=str, help='directory where the relative comparison summary will be output')

    (options, args) = parser.parse_args()

    if not options.folder_paths:
        parser.error('Folder paths not given')

    summary_data = read_test_summary_files(options.folder_paths)
    if summary_data:
        test_summary_by_module = summarize_test_summary_by_module(summary_data)
        test_summary_by_case = summarize_test_summary_by_case(summary_data)
        print_test_summary(test_summary_by_module, True)
        print_test_summary(test_summary_by_case, False)
        if options.output_dir:
            output_summary_to_csv(test_summary_by_module, options.output_dir, 'test_summary_by_module_comparison.csv')
            output_summary_to_csv(test_summary_by_case, options.output_dir, 'test_summary_by_case_comparison.csv')

    evaluation_data = read_test_evaluation_files(options.folder_paths)
    if evaluation_data:
        test_evaluation_by_module = summarize_test_evaluation_by_module(evaluation_data)
        test_evaluation_by_case = summarize_test_evaluation_by_case(evaluation_data)
        print_test_evaluation(test_evaluation_by_module, True)
        print_test_evaluation(test_evaluation_by_case, False)
        if options.output_dir:
            output_summary_to_csv(test_evaluation_by_module, options.output_dir, 'test_evaluation_by_module_comparison.csv')
            output_summary_to_csv(test_evaluation_by_case, options.output_dir, 'test_evaluation_by_case_comparison.csv')


if __name__ == "__main__":
    main()
