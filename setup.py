#!/usr/bin/env python

from setuptools import setup, find_packages
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='cnspy_ranging_evaluation',
    version="0.2.14",
    author='Roland Jung',
    author_email='roland.jung@aau.at',
    description='Evaluation of range measurements.',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url='https://github.com/aau-cns/cnspy_ranging_evaluation/',
    project_urls={
        "Bug Tracker": "https://github.com/aau-cns/cnspy_ranging_evaluation/issues",
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],

    packages=find_packages(exclude=["test_*", "TODO*"]),
    python_requires='>=3.6',
    install_requires=['numpy',
                      'pandas',
                      'spatialmath-python',
                      'scipy',
                      'matplotlib',
                      'joblib',
                      'configparser',
                      'cnspy_numpy_utils',
                      'cnspy_timestamp_association',
                      'cnspy_trajectory>=0.2.14'],
    entry_points={
        'console_scripts': [
            'CSV_StaticBiasAnalysis = cnspy_ranging_evaluation.CSV_StaticBiasAnalysis:main',
            'IMU_ROSbag2CSV = cnspy_ranging_evaluation.IMU_ROSbag2CSV:main',
            'RangeEvaluationTool = cnspy_ranging_evaluation.RangeEvaluationTool:main',
            'ROSBag_Pose2Ranges = cnspy_ranging_evaluation.ROSBag_Pose2Ranges:main',
            'ROSBag_TrueRanges = cnspy_ranging_evaluation.ROSBag_TrueRanges:main',
            'TWR_ROSbag2CSV = cnspy_ranging_evaluation.TWR_ROSbag2CSV:main',
            'PlotTwoWayRanges = cnspy_ranging_evaluation.PlotTwoWayRanges:main',
        ],
    },
)



