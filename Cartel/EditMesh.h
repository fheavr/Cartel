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

#include <Eigen/Core>

#include <cstdlib>
#include <vector>

//#define USE_PREV

const std::size_t HOLE_INDEX = static_cast<std::size_t>( -1 );

struct half_edge{
	std::size_t next; // Index of next half-edge in the loop.
#ifdef USE_PREV
	std::size_t prev; // Index of the previous half-edge in the loop. Alternatively: use next->next if you have strictly triangle meshes.
#endif
	std::size_t twin; // Index of half-edge that is in other face that shares this edge. Open edges?
	std::size_t vert; // Index of vertex at the start of this edge.
	std::size_t face; // Index of face to the "left" of this edge.
};

class EditMesh;

/* an iterator over the one ring of vertices pointed to by this
 * half edge */
class vvert_iterator{
private:
	friend class EditMesh;
	const half_edge* m_cur;
	const half_edge* m_end;
};

class vface_iterator{
private:
	friend class EditMesh;
	const half_edge* m_cur;
	const half_edge* m_end;
	const half_edge* m_next;
};

class fvert_iterator{
private:
	friend class EditMesh;
	const half_edge* m_cur;
	const half_edge* m_end;
};

class fface_iterator{
private:
	friend class EditMesh;
	const half_edge* m_cur;
	const half_edge* m_end;
};

class EditMesh{
public:
	EditMesh();

	/**
	 * Initialize the mesh from existing data.
	 * \param xyzPositions A list of doubles, storing the vertex data interleaved (ie. X1,Y1,Z1,X2,Y2,Z2,etc.)
	 * \param triangleVerts A list of vertex indices, where each run of 3 defines a triangle in the mesh. (ie. T1.v1, T1.v2, T1.v3, T2.v1, T2.v2, T2.v3, etc.)
	 */
	void init( const std::vector<double>& xyzPositions, const std::vector<std::size_t>& triangleVerts );
	void clear();

	std::size_t add_vertex( double x, double y, double z );
	std::size_t add_face( std::size_t v1, std::size_t v2, std::size_t v3 );
	std::size_t add_face( std::size_t (&f)[3] );
	
	void delete_face( std::size_t f );
	std::size_t collapse_edge( std::size_t v1, std::size_t v2 );
	
	std::size_t split_face_center( std::size_t f, std::size_t (*pOutFaceIndices)[3] = NULL );

    /**
     * returns the position of a vertex
     * \param i the vertex index (not the he index)
     * \return Vector3d the vertex position values
     */
	Eigen::Vector3d get_vertex( std::size_t i ) const;
    Eigen::Vector3d get_vnormal( std::size_t i ) const;
    Eigen::Vector3d get_fnormal( std::size_t i ) const;

	void set_vertex( std::size_t i, const Eigen::Vector3d& v );
	
    /************************************
     * Mesh modification functions
     ************************************/
    void subdivide_sqrt3(std::size_t num_steps = 1);

    /**
     * Given a vertex, calculate its one ring and flatten this one ring to 2d
     * \param vert_id  the id of the vertex around which to operate
     * \param v_ring   the address of the vector to store the one ring
     * \param v_ring_flat the address of the vector to store the flattened one ring (with the center vert at 0)
     * \return returns false if the vertex is on an edge and true otherwise
     */
    bool remesh_flatten(std::size_t vert_id,
                        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &v_ring,
                        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &v_ring_flat);
    /**
     * Given a vertex, its new position in 2d, its flat and 3d one-rings, find out its new location in 3d
     * \param orig_p   the original vertex position
     * \param pt_2d    the new vertex position in 2d
     * \param v_ring   the 3-dimensional one-ring
     * \param v_ring_flat the 2-dimensional one-ring
     * \return  returns the location of the new vert in 3d (or the original point if outside of the umbrella)
     */
    Eigen::Vector3d remesh_unflatten(Eigen::Vector3d &orig_p, Eigen::Vector2d &pt_2d,
                        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &v_ring,
                        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &v_ring_flat);

    Eigen::Vector3d remesh_relocateVertDelaunay(std::size_t vert_id);
    Eigen::Vector3d remesh_relocateVert(std::size_t vert_id);
    Eigen::Vector3d remesh_areaBased(std::size_t vert_id);
    void            remesh_delaunayize();
    void remesh_relocate(std::size_t num_steps = 1);
    void remesh_area(std::size_t num_steps = 1);
    void remesh();
    void remesh_pretty();

	/**
	 * Initialize an iterator that visits the 1-ring of 'vertex'.
	 * \param it The iterator to initialize.
	 * \param vertex The index of the vertex to iterate around.
	 * \return False if the vertex has no neighbors (ie. a floating vertex)
	 */
	bool init_iterator( vvert_iterator& it, std::size_t vertex ) const;
	
    /**
     * move the iterator to a boundary, if one exists, and set its end
     * to the current location
     * \param it The iterator to move
     * \return False if there is no boundary
     */
    bool reset_boundary_iterator( vvert_iterator& it ) const;

    /**
     * \param face index
     * \return true if on boundary false else
     */
    bool isBoundaryFace( std::size_t i ) const;

	/**
	 * Advances a vertex iterator to the next vertex in the 1-ring.
	 * \param it The iterator to advance
	 * \return False if the iterator has completed the loop. It may continue to be used at this point as it will merely restart the loop.
	 */
	bool advance_iterator( vvert_iterator& it ) const;
	
	std::size_t deref_iterator( const vvert_iterator& it ) const;
	std::size_t deref_iterator_left_face( const vvert_iterator& it ) const;
	std::size_t deref_iterator_right_face( const vvert_iterator& it ) const;
	std::size_t deref_iterator_left_edge( const vvert_iterator& it ) const;
	std::size_t deref_iterator_right_edge( const vvert_iterator& it ) const;

	Eigen::Vector3d get_vertex( const vvert_iterator& it ) const;
	double get_cotan_weight( const vvert_iterator& it ) const;
	double get_mean_value_weight( const vvert_iterator& it ) const;

	/**
	 * Intialize an iterator that visits the faces in the 1-ring of 'vertex'
	 */
	bool init_iterator(vface_iterator& it, std::size_t vertex ) const;
	bool advance_iterator( vface_iterator& it ) const;
	std::size_t deref_iterator( const vface_iterator& it ) const;
	Eigen::Vector3d get_normal( const vface_iterator& it ) const;
	Eigen::Vector4d get_plane( const vface_iterator& it ) const;

	bool init_iterator(fvert_iterator& it, std::size_t face ) const;
	bool advance_iterator( fvert_iterator& it ) const;
	std::size_t deref_iterator( const fvert_iterator& it ) const;

	bool init_iterator(fface_iterator& it, std::size_t face ) const;
	bool advance_iterator( fface_iterator& it ) const;
	std::size_t deref_iterator( const fface_iterator& it ) const;

    /* get a list of all vertices/face indices in the mesh as floats in contiguous
     * memory. Used for rendering so loss of precision unimportant */
    void get_draw_data( float *verts, int *indices ) const;
    void get_draw_normals( float *normals ) const;
    int  get_edit_count() const;
	void flag_editted();

    /* get information about internal state */
    std::size_t get_vert_size() const;
    std::size_t get_face_size() const;

    void test_flip();
	static void test();
	void verify() const;

	void write_to_obj_stream( std::ostream& stream ) const;

private:
	const half_edge& prev( const half_edge& cur ) const;
	const half_edge& next( const half_edge& cur ) const;
	const half_edge& twin( const half_edge& cur ) const;

	half_edge* find_edge( std::size_t vFrom, std::size_t vTo );
	half_edge* find_twin( std::size_t vFrom, std::size_t vTo );
	std::size_t collapse_edge( std::size_t he );

	void delete_half_edge_impl( std::size_t he );

	template <std::size_t N>
	void delete_half_edges_impl( std::size_t (&edges)[N] );
	
	// Splits a boundary edge into 3 by adding 2 vertices on the specified edge, connecting them to the other vertex. Replaces this face with 3 new ones.
	void split_boundary_edge( std::size_t he, std::size_t (*pOutVertIndices)[2] = NULL, std::size_t (*pOutFaceIndices)[3] = NULL );

    /**
     * a simple helper function to determine if adding a particular face
     * will break mesh manifoldness
     * \param input vertices that could become a face
     * \return if this face will break manifoldness
     */
    bool is_safe_addface( std::size_t v1, std::size_t v2, std::size_t v3 );
    /**
     * Flip the passed in edge to connect the two opposing vertices
     * Flips both the passed in half_edge and its twin
     * \param cur The half edge to flip
     * \return True if successfully flipped, false if edge
     */
    bool flip_edge( half_edge &cur );

private:
    /*****************************************
     * Filter Used Variables
     *****************************************/
    unsigned int subdiv_iter; // used to track even applications of subdivision

    /*****************************************
     * Mesh Variables
     *****************************************/
	friend class mesh_adjacency;
    int edit_count; // increment this value every change

	// TODO: Switch to std::deque to avoid pointer invalidation when adding new half-edges.
	std::vector< half_edge > m_heData;     // All the half-edges that make up the mesh.
	std::vector< std::size_t > m_faceData; // A mapping from face index to an arbitrary half-edge on its boundary.
	std::vector< std::size_t > m_vertData; // A mapping from vertex index to an arbitrary half-edge originating from this vertex. Can be "HOLE_INDEX" for unconnected vertices.
	
	std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_vertices;
	std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_simplifyData;
	
	struct node{
		double value;
	};

	std::vector<node> m_simplifyQueue;

	// TODO: If we need arbitrary data, add a std::map< std::string, vector > that holds the per-vertex data. 
	// TODO: Same for per-half-edge and per-face data.
	friend void collapse_one_edge( EditMesh& );
};

void collapse_one_edge( EditMesh& m );

inline EditMesh::EditMesh() 
	: edit_count(0), subdiv_iter(0)
{}

inline void EditMesh::clear(){
	subdiv_iter = 0;
	edit_count = 0;

	m_heData.clear();
	m_faceData.clear();
	m_vertData.clear();
	m_vertices.clear();
}

inline std::size_t EditMesh::add_vertex( double x, double y, double z ){
	std::size_t newIndex = m_vertices.size();
	m_vertices.emplace_back( x,y,z );
	m_vertData.push_back( HOLE_INDEX );
	return newIndex;
}

inline Eigen::Vector3d EditMesh::get_vertex( std::size_t i ) const {
	return m_vertices[i];
}

inline void EditMesh::set_vertex( std::size_t i, const Eigen::Vector3d& v ) {
	m_vertices[i] = v;
}

inline std::size_t EditMesh::add_face( std::size_t v1, std::size_t v2, std::size_t v3 ) {
	std::size_t v[] = { v1, v2, v3 };
	return this->add_face( v );
}

inline std::size_t EditMesh::collapse_edge( std::size_t v1, std::size_t v2 ){
	return this->collapse_edge( this->find_twin( v1, v2 )->twin );
}

inline bool EditMesh::init_iterator( vvert_iterator& it, std::size_t vertex ) const {
	std::size_t vertToHE = m_vertData[ vertex ];
			
	if( vertToHE == HOLE_INDEX )
		return false;

	// Store a pointer to the (arbitrary) first half-edge pointing into the specified vertex. This implies m_heData[m_cur->twin].vert == vertex & m_heData[m_cur->next].vert == vertex.
	// By iterating around the half-edges pointing into 'vertex' we will visit all the vertices in the 1-ring.
	it.m_cur = it.m_end = &m_heData[ m_heData[ vertToHE ].twin ];
	return true;
}

inline bool EditMesh::reset_boundary_iterator( vvert_iterator &it ) const {
   const half_edge *cur = it.m_cur;
    do {
        if (it.m_cur->face == HOLE_INDEX)
        {
            it.m_end = it.m_cur;
            return true;
        }
        it.m_cur = &m_heData[ m_heData[ it.m_cur->next ].twin ];
    } while (cur != it.m_cur);

    return false;
}

inline bool EditMesh::isBoundaryFace( std::size_t i ) const {
    std::size_t he_index = m_faceData[i];
    const half_edge *he = &m_heData[ he_index ];
    while( he->next != he_index ) {
        if( m_heData[he->twin].face == HOLE_INDEX )
            return true;
        he = &m_heData[ he->next ];
    }
    return false;
}

inline bool EditMesh::advance_iterator( vvert_iterator& it ) const {
	it.m_cur = &m_heData[ m_heData[ it.m_cur->next ].twin ];
	return it.m_cur != it.m_end;
}

inline std::size_t EditMesh::deref_iterator( const vvert_iterator& it ) const {
	return it.m_cur->vert;
}

inline std::size_t EditMesh::deref_iterator_left_face( const vvert_iterator& it ) const {
	return m_heData[it.m_cur->twin].face;
}

inline std::size_t EditMesh::deref_iterator_right_face( const vvert_iterator& it ) const {
	return it.m_cur->face;
}

inline std::size_t EditMesh::deref_iterator_left_edge( const vvert_iterator& it ) const {
	return it.m_cur->twin;
}

inline std::size_t EditMesh::deref_iterator_right_edge( const vvert_iterator& it ) const {
	return m_heData[ it.m_cur->twin ].twin;
}

inline bool EditMesh::init_iterator( vface_iterator& it, std::size_t vertex ) const {
	std::size_t vertToHE = m_vertData[ vertex ];
			
	if( vertToHE == HOLE_INDEX )
		return false;

	// Store a pointer to the (arbitrary) first half-edge pointing into the specified vertex. This implies m_heData[m_cur->twin].vert == vertex & m_heData[m_cur->next].vert == vertex.
	// By iterating around the half-edges pointing into 'vertex' we will visit all the vertices in the 1-ring.
	it.m_cur = it.m_end = &m_heData[ m_heData[ vertToHE ].twin ];
	it.m_next = &m_heData[ m_heData[ it.m_cur->next ].twin ];
	return true;
}

inline bool EditMesh::advance_iterator( vface_iterator& it ) const {
	it.m_cur = it.m_next;
	it.m_next = &m_heData[ m_heData[ it.m_next->next ].twin ];
	return it.m_cur != it.m_end;
}

inline std::size_t EditMesh::deref_iterator( const vface_iterator& it ) const {
	return it.m_cur->face;
}

inline bool EditMesh::init_iterator(fvert_iterator& it, std::size_t face ) const {
	assert( face < m_faceData.size() );
	std::size_t faceToHE = m_faceData[ face ];
	assert( faceToHE < m_heData.size() );

	// Store a pointer to the (arbitrary) first half-edge pointing into the specified vertex. This implies m_heData[m_cur->twin].vert == vertex & m_heData[m_cur->next].vert == vertex.
	// By iterating around the half-edges pointing into 'vertex' we will visit all the vertices in the 1-ring.
	it.m_cur = it.m_end = &m_heData[ faceToHE ];
	return true;
}

inline bool EditMesh::advance_iterator( fvert_iterator& it ) const {
	it.m_cur = &m_heData[ it.m_cur->next ];
	return it.m_cur != it.m_end;
}

inline std::size_t EditMesh::deref_iterator( const fvert_iterator& it ) const {
	return it.m_cur->vert;
}

inline bool EditMesh::init_iterator(fface_iterator& it, std::size_t face ) const {
	assert( face < m_faceData.size() );
	std::size_t faceToHE = m_faceData[ face ];
	assert( faceToHE < m_heData.size() );

	// Store a pointer to the (arbitrary) first half-edge pointing into the specified vertex. This implies m_heData[m_cur->twin].vert == vertex & m_heData[m_cur->next].vert == vertex.
	// By iterating around the half-edges pointing into 'vertex' we will visit all the vertices in the 1-ring.
	it.m_cur = it.m_end = &m_heData[ faceToHE ];
	return true;
}

inline bool EditMesh::advance_iterator( fface_iterator& it ) const {
	it.m_cur = &m_heData[ it.m_cur->next ];
	return it.m_cur != it.m_end;
}

inline std::size_t EditMesh::deref_iterator( const fface_iterator& it ) const {
	return m_heData[ it.m_cur->twin ].face;
}

inline const half_edge& EditMesh::prev( const half_edge& cur ) const {
#ifdef USE_PREV
	return m_heData[ cur.prev ];
#else
	return m_heData[ m_heData[ cur.next ].next ];
#endif
}

inline const half_edge& EditMesh::next( const half_edge& cur ) const {
	return m_heData[ cur.next ];
}

inline const half_edge& EditMesh::twin( const half_edge& cur ) const {
	return m_heData[ cur.twin ];
}

inline std::size_t EditMesh::get_vert_size() const {
    return m_vertData.size();
}

inline std::size_t EditMesh::get_face_size() const {
    return m_faceData.size();
}

inline int EditMesh::get_edit_count() const{
    return edit_count;
}

inline void EditMesh::flag_editted(){
	++edit_count;
}