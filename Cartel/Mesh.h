/* Copyright (c) Darcy Harisson, Russell Gillette
 * April 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#pragma once
#ifndef OBJECT_MESH_H
#define OBJECT_MESH_H

#include "DrawMesh.h"
#include "EditMesh.h"

class Mesh
{
public:
    // NOTE: we take ownership of the EditMesh
    Mesh::Mesh(EditMesh &em);
    Mesh::~Mesh();

    void init(RenderState &state);

    void drawMesh();
    void subdivide_sqrt3();
    void remesh_relocate();
    void remesh_areabased();
    void remesh_delaunay();
    void remesh();
    void remesh_pretty();
	void collapse_edge();

	const EditMesh& get_edit_mesh() const;

private:
    void updateDrawMesh();

private:
    //indicates the last drawn edit mesh, if this does not match
    // the value in m_em then the draw mesh needs to be updated
    int last_drawn;

    DrawMesh m_dm;
    EditMesh *m_em;
};

inline const EditMesh& Mesh::get_edit_mesh() const {
	return *m_em;
}

inline void Mesh::subdivide_sqrt3() {
    m_em->subdivide_sqrt3();
}

inline void Mesh::remesh_relocate() {
    m_em->remesh_relocate();
}

inline void Mesh::remesh_areabased() {
    m_em->remesh_area();
}

inline void Mesh::remesh_delaunay() {
    m_em->remesh_delaunayize();
}

inline void Mesh::remesh() {
    m_em->remesh();
}

inline void Mesh::remesh_pretty() {
    m_em->remesh_pretty();
}

inline void Mesh::collapse_edge() {
	collapse_one_edge( *m_em );
}

#endif //OBJECT_MESH_H