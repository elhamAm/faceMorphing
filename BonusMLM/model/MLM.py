import torch
import tensorly as tl
from tensorly.decomposition import partial_tucker
import os
import sys


from helpers import *

class MLM():
    '''
    Multilinear Model based on tensorly. Decomposes mesh tensor with partial_tucker method.
        * Mode 1 = Vertices (flatten)
        * Mode 2 = Expressions
        * Mode 3 = Identities
    '''

    def __init__(self):
        print("Multilinear Model")

    def load(self, path):
        ''' Loads a precomputed tucker decomposition '''
        print("load...")
        self.dataset = torch.load(os.path.join(path, "dataset.pt"))
        self.dataset_mean = torch.load(os.path.join(path, "dataset_mean.pt"))

        self.core = torch.tensor(torch.load(os.path.join(path, "core.pt")))
        self.factor_exp = torch.tensor(torch.load(os.path.join(path, "factor_exp.pt")))
        self.factor_id = torch.tensor(torch.load(os.path.join(path, "factor_id.pt")))

        self.indices = torch.tensor(torch.load(os.path.join(path, "indices.pt")))
        print("done!")

    def save(self, path):
        ''' Save a computed tucker decomposition '''
        print("save...")
        torch.save(self.dataset, os.path.join(path, "dataset.pt"))
        torch.save(self.dataset_mean, os.path.join(path, "dataset_mean.pt"))

        torch.save(self.core, os.path.join(path, "core.pt"))
        torch.save(self.factor_exp, os.path.join(path, "factor_exp.pt"))
        torch.save(self.factor_id, os.path.join(path, "factor_id.pt"))
        torch.save(self.indices, os.path.join(path, "indices.pt"))
        print("done!")

    def compute(self, dataset, indices, exp_knobs, id_knobs):
        ''' computes a tucker decomposition given its data and dimensionality '''
        print("compute...")
        self.indices = indices
        self.dataset_mean = torch.mean(dataset, [1, 2])
        self.dataset = dataset
        datasetM = dataset - torch.tile(torch.reshape(self.dataset_mean,(self.dataset_mean.shape[0],1,1)), (1, dataset.shape[1], dataset.shape[2]))

        # compute tucker decompositions for expression and identity
        core, factors = partial_tucker(dataset, modes=[1, 2], rank=[exp_knobs, id_knobs])
        self.core = torch.tensor(core, requires_grad=True)
        self.factor_exp = torch.tensor(factors[0], requires_grad=True)
        self.factor_id = torch.tensor(factors[1], requires_grad=True)
        print("done!")

    def reconstruct(self, factor_exp, factor_id):
        recon = tl.tenalg.mode_dot(tl.tenalg.mode_dot(self.core, factor_exp, 1), factor_id, 2)
        return recon #+ self.dataset_mean.reshape((self.dataset_mean.shape[0], 1, 1))