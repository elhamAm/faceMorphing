import torch
import numpy as np
import tensorly as tl
import trimesh

def unflat_mesh(mesh_vector):
    mesh = torch.zeros(mesh_vector.shape[0]//3, 3)

    for i in range(mesh.shape[0]):
        mesh[i, 0] = mesh_vector[3*i]
        mesh[i, 1] = mesh_vector[3*i+1]
        mesh[i, 2] = mesh_vector[3*i+2]

    return mesh


def flat_mesh(mesh):
    mesh_vector = torch.zeros(mesh.shape[0]*3)

    for i in range(mesh.shape[0]):
        mesh_vector[3 * i] = mesh[i, 0]
        mesh_vector[3 * i+1] = mesh[i, 1]
        mesh_vector[3 * i+2] = mesh[i, 2]

    return mesh_vector


def decode_indices(indices):
    ind_to_pos = {}
    pos_to_ind = {}

    indices = indices.reshape(indices.shape[0] * 3)
    indices_default = np.zeros_like(indices)

    pos = 0
    for i in range(indices.shape[0]):
        coming = indices[i]
        if coming not in ind_to_pos:
            ind_to_pos[coming] = pos
            pos_to_ind[pos] = coming
            pos += 1

        change = ind_to_pos[coming]
        indices_default[i] = change

    return [ind_to_pos, pos_to_ind, indices_default.reshape((indices_default.shape[0] // 3, 3))]

def resort_vertices(vertices, pos_to_ind):
    resorted = np.zeros_like(vertices)
    for p in range(resorted.shape[0]):
        from_here = pos_to_ind[p]
        resorted[p] = vertices[from_here]
    return resorted

def load_mesh(path):
    meshdata = trimesh.load(path)
    decoding = decode_indices(meshdata.faces)
    meshdata = resort_vertices(meshdata.vertices, decoding[1])
    #meshdata = pywavefront.Wavefront(path)
    mesh = tl.tensor(torch.zeros(meshdata.shape[0]*3))
    for i in range(meshdata.shape[0]):
        mesh[i*3] = meshdata[i, 0]
        mesh[i * 3 + 1] = meshdata[i,1]
        mesh[i * 3 + 2] = meshdata[i,2]
    return mesh, decoding[2]

def save_mesh(mesh, path):
    f = open(path, "w")
    f.write(trimesh.exchange.obj.export_obj(mesh))
    f.close()