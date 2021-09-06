## Requirements
We created a conda environment with all the dependencies needed.

`conda env create -f environment.yml `

## Playground
We created a Jupyter notebook which can be easily used to play around with the MLM. The MLM is precomputed and only has to be loaded. Checkout `MLM_Playground.ipynb`

## Compute MLM
The environment allows also for computing an MLM from the data. This is a step-by-step guide how to this:
1. ``preprocess.py`` has implemented two different versions of data preprocessing.
* ``preprocess_reconstruction_version()``: creates a MLM from the whole dataset and does not skip any subject. Additionally, it keeps the highest degree of freedom by providing the maximal numbers of knobs for identity and expression.
* ``preprocess_missingData_version()``: this version of the MLM leaves out a test subject where we swap the smiling expression with the neutral. Later this model can show that we can fill in the missing expression of this subject by using the expression code for smiling and the identity code of the subject.


