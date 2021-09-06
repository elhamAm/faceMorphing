"""
this script loads the data in puts it in the matrix form needed for the MLM
"""
import os

import tensorly as tl
tl.set_backend("pytorch")


from model import MLM
from helpers import *

def get_mesh_info(name):
    return name.split(".")[0].split("_")

def preprocess_reconstruction_version(data_path, expressions, identities, exp_knobs, id_knobs, expression_codes, identity_codes, vertices, save_path):
    os.makedirs(save_path, exist_ok=True)
    FACES = torch.zeros((vertices, expressions, identities))

    for mesh_name in sorted(os.listdir(data_path)):
        mesh_path = os.path.join(data_path, mesh_name)
        mesh_info = get_mesh_info(mesh_name)

        if mesh_info[0] not in identity_codes:
            continue

        if mesh_info[1] not in expression_codes:
            continue

        id_code = identity_codes[mesh_info[0]]
        exp_code = expression_codes[mesh_info[1]]

        mesh, indices = load_mesh(mesh_path)

        # register mesh to matrix
        FACES[:, exp_code, id_code] = mesh

        # name_id_exp.obj
        print(mesh_path)
        print(exp_code)
        print(id_code)

    # compute MLM
    mlm = MLM()
    mlm.compute(FACES, indices, exp_knobs, id_knobs)
    mlm.save(save_path)

def preprocess_missingData_version(data_path, expressions, identities, exp_knobs, id_knobs, expression_codes, identity_codes, vertices, save_path, expression, subject):
    os.makedirs(save_path, exist_ok=True)
    FACES = torch.zeros((vertices, expressions, identities))

    mean_mesh = 0
    for mesh_name in sorted(os.listdir(data_path)):
        mesh_path = os.path.join(data_path, mesh_name)
        mesh_info = get_mesh_info(mesh_name)

        if mesh_info[0] not in identity_codes:
            continue

        if mesh_info[1] not in expression_codes:
            continue

        id_code = identity_codes[mesh_info[0]]
        exp_code = expression_codes[mesh_info[1]]

        mesh, indices = load_mesh(mesh_path)

        # register mesh to matrix
        FACES[:, exp_code, id_code] = mesh
        if exp_code == 1:
            mean_mesh += mesh

        # name_id_exp.obj
        print(mesh_path)
        print(exp_code)
        print(id_code)

    mean_mesh /= len(os.listdir(data_path))
    mean_mesh *= 2

    FACES[:, expression, subject] = mean_mesh

    # compute MLM
    mlm = MLM()
    mlm.compute(FACES, indices, exp_knobs, id_knobs)
    mlm.save(save_path)
    print(identity_codes)


if __name__ == '__main__':
    # set parameters
    data_path = "../allTasksExceptLandmarks/data/nonRigidFaces"

    expression_codes = {"neutral": 0, "normal": 0, "smile":1}
    identity_pre = []
    identity_codes = {}
    expressions = 2
    identities = 0
    get_vertex = True
    for p in sorted(os.listdir(data_path)):
        mesh_info = get_mesh_info(p)
        id_code = mesh_info[0]
        exp_code = mesh_info[1]

        if exp_code not in expression_codes:
            continue

        if id_code not in identity_pre:
            identity_pre.append(id_code)
            continue
        else:
            identity_codes[id_code] = identities
            identities += 1

        if get_vertex:
            get_vertex = False
            mesh_path = os.path.join(data_path, p)
            mesh, indices = load_mesh(mesh_path)
            num_vertex = mesh.shape[0]


    vertices = num_vertex


    preprocess_reconstruction_version(data_path,
                                      expressions = expressions,
                                      identities = identities,
                                      exp_knobs = expressions,
                                      id_knobs = identities,
                                      expression_codes = expression_codes,
                                      identity_codes = identity_codes,
                                      vertices = vertices,
                                      save_path = "model/precomputeReconstruction")

    preprocess_missingData_version(data_path,
                                   expressions=expressions,
                                   identities=identities,
                                   exp_knobs=expressions-1,
                                   id_knobs=identities-10,
                                   expression_codes=expression_codes,
                                   identity_codes=identity_codes,
                                   vertices=vertices,
                                   save_path="model/precomputeMissingData",
                                   subject=9,
                                   expression=1)

