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
#define _SCL_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include "EditMesh.h"

#include <Eigen/Geometry>
#include <cmath>

#include <map>
#include <set>
#include <iostream>
#include <memory>
#include <fstream>

//#if defined(NDEBUG) && defined(ALWAYS_ASSERT)
//#undef NDEBUG
//#endif
//#include <cassert>

EditMesh *loadEditMeshFromFile(std::string file_name);

namespace detail{
	inline void init( half_edge& he, std::size_t next, std::size_t twin, std::size_t vert, std::size_t face ){
		he.next = next;
		he.twin = twin;
		he.vert = vert;
		he.face = face;
	}

	void delete_face( std::vector<std::size_t>& faceData, std::vector<half_edge>& heData, std::size_t f ){
		assert( f < faceData.size() );

		// In order to delete the face properly, we need to move a face from the end of the list to overwrite 'f'. Then we need to update the 
		// indices stored in the moved face's half-edges.
		faceData[f] = faceData.back();
		faceData.pop_back();

		if( f != faceData.size() ){
			//std::clog << "Reindexed face " << faceData.size() << " to " << f << std::endl;

			std::size_t he = faceData[f];
			do {
				assert( heData[he].face == faceData.size() );

				heData[he].face = f;
				he = heData[he].next;
			} while( he != faceData[f] );
		}
	}

	template <int N>
	void delete_faces( std::vector<std::size_t>& faceData, std::vector<half_edge>& heData, std::size_t (&fToDelete)[N] ){
		// Sort the faces by decreasing index so that we can safely delete them all without causing any of them to be accidentally re-indexed (which
		// cause 'fToDelete' to contain invalid indices). This also chooses the optimal deletion order to minimize re-indexing.
		std::sort( fToDelete, fToDelete + N, std::greater<std::size_t>() );
		for( std::size_t i = 0; i < N; ++i )
			detail::delete_face( faceData, heData, heToDelete[i] );
	}
}

void init_adjacency( std::size_t numVertices, const std::vector<std::size_t>& faces, std::vector< half_edge >& m_heData, std::vector< std::size_t >& m_faceData, std::vector< std::size_t >& m_vertData ){
	typedef std::map< std::pair<std::size_t, std::size_t>, std::size_t > edge_map_type;
	
	assert( faces.size() % 3 == 0 && "Invalid data specified for faces. Must have 3 vertex indices per face." );

	edge_map_type edgeMap; // Use a temporary map to find edge pairs.

	m_heData.reserve( faces.size() ); // Assume there are 3 edges per face.
	m_faceData.resize( faces.size() / 3 );
	m_vertData.resize( numVertices, HOLE_INDEX ); // Init with HOLE_INDEX since a vert might be floating w/ no faces.

	for( std::size_t i = 0, iEnd = faces.size(); i < iEnd; i+=3 ){
		std::size_t f[] = { faces[i], faces[i+1], faces[i+2] };
		std::size_t fIndex = i / 3;

		// The index of the first (of three) half-edges associated with the current face.
		std::size_t heIndex = m_heData.size();

		half_edge he[3];
		detail::init( he[0], heIndex+1, HOLE_INDEX, f[0], fIndex );
		detail::init( he[1], heIndex+2, HOLE_INDEX, f[1], fIndex );
		detail::init( he[2], heIndex, HOLE_INDEX, f[2], fIndex );
#ifdef USE_PREV
		he[0].prev = heIndex+2;
		he[1].prev = heIndex;
		he[2].prev = heIndex+1;
#endif
			
		// These will be set each time a vertex is referenced, but that's fine. The last assignment will stick.
		m_faceData[ fIndex ] = heIndex;
		m_vertData[ f[0] ] = heIndex;
		m_vertData[ f[1] ] = heIndex+1;
		m_vertData[ f[2] ] = heIndex+2;

		edge_map_type::iterator it;

		it = edgeMap.lower_bound( std::make_pair( f[0], f[1] ) );
		if( it != edgeMap.end() && it->first.first == f[0] && it->first.second == f[1] ){
			m_heData[it->second].twin = heIndex;
			he[0].twin = it->second;
			edgeMap.erase( it );
		}else{
			he[0].twin = HOLE_INDEX;
			edgeMap.insert( it, std::make_pair( std::make_pair( f[1], f[0] ), heIndex ) ); // NOTE: Reversed order since we are matching opposite half_edge.
		}

		it = edgeMap.lower_bound( std::make_pair( f[1], f[2] ) );
		if( it != edgeMap.end() && it->first.first == f[1] && it->first.second == f[2] ){
			m_heData[it->second].twin = heIndex+1;
			he[1].twin = it->second;
			edgeMap.erase( it );
		}else{
			he[1].twin = HOLE_INDEX;
			edgeMap.insert( it, std::make_pair( std::make_pair( f[2], f[1] ), heIndex+1 ) ); // NOTE: Reversed order since we are matching opposite half_edge.
		}

		it = edgeMap.lower_bound( std::make_pair( f[2], f[0] ) );
		if( it != edgeMap.end() && it->first.first == f[2] && it->first.second == f[0] ){
			m_heData[it->second].twin = heIndex+2;
			he[2].twin = it->second;
			edgeMap.erase( it );
		}else{
			he[2].twin = HOLE_INDEX;
			edgeMap.insert( it, std::make_pair( std::make_pair( f[0], f[2] ), heIndex+2 ) ); // NOTE: Reversed order since we are matching opposite half_edge.
		}

		m_heData.push_back( he[0] );
		m_heData.push_back( he[1] );
		m_heData.push_back( he[2] );
	}

	// Keep track of the last edge we processed so we can hook up half_edge::prev as we go.
	std::size_t prev = HOLE_INDEX;

	// Add half-edges for any holes. Any edges still in the map are holes.
	edge_map_type::iterator it = edgeMap.begin();
	while( it != edgeMap.end() ){
		half_edge he;
		detail::init( he, HOLE_INDEX, it->second, it->first.first, HOLE_INDEX );
#ifdef USE_PREV
		he.prev = prev;
		prev = m_heData.size(); // Size is the index of the half_edge we are about to push into the list.
#endif

		m_heData[he.twin].twin = m_heData.size();
		m_heData.push_back( he );

		std::size_t curVert = it->first.first;
		std::size_t nextVert = it->first.second; // We are about to erase this information, so store it to use later.

		edgeMap.erase( it ); // We are done with this edge now.

		half_edge* twinPrev = &m_heData[m_heData[m_heData[he.twin].next].next];
		while( twinPrev->twin != HOLE_INDEX && m_heData[twinPrev->twin].face != HOLE_INDEX ){
			assert( m_heData[twinPrev->next].vert == nextVert );
			assert( m_heData[twinPrev->twin].vert == nextVert );
			twinPrev = &m_heData[m_heData[m_heData[twinPrev->twin].next].next];
		}

		if( twinPrev->twin == HOLE_INDEX ){
			// We haven't processed the next edge in the loop yet. Let's do so now so we can assume the index of the next half-edge.
			m_heData.back().next = m_heData.size();
			it = edgeMap.find( std::make_pair( nextVert, twinPrev->vert ) );
				
			assert( it != edgeMap.end() );
		}else{
			assert( m_heData[twinPrev->twin].vert == nextVert );
			assert( m_heData[twinPrev->twin].face == HOLE_INDEX );

			// We already processed this edge and have a valid index for the next half_edge.
			m_heData.back().next = twinPrev->twin;
#ifdef USE_PREV
			m_heData[ twinPrev->twin ].prev = prev; // Complete the loop
			prev = HOLE_INDEX;
#endif
			it = edgeMap.begin(); // Arbitrarily pick the next edge in the list.
		}
	}

	assert( edgeMap.empty() );
}

void EditMesh::init( const std::vector<double>& xyzPositions, const std::vector<std::size_t>& triangleVerts ){
	assert( xyzPositions.size() % 3 == 0 && "Invalid vertex positions for EditMesh::init(). Must have 3 values per-vertex." );
	assert( triangleVerts.size() % 3 == 0 && "Invalid face data for EditMesh::init(). Must have 3 vertex indices per face." );

	//m_vertices.resize( Eigen::NoChange, xyzPositions.size() / 3 );
	m_vertices.resize( xyzPositions.size() / 3 );

	// The Eigen matrix has the same format as the incoming vector so we can straight copy it.
	// HACK: This is pretty sketchy and relies on Eigen::Vector3d having the same layout as a double[3] and nothing extra or fancy alignment.
	std::copy( xyzPositions.begin(), xyzPositions.end(), m_vertices.front().data() );

	init_adjacency( xyzPositions.size() / 3, triangleVerts, m_heData, m_faceData, m_vertData );
}

half_edge* EditMesh::find_twin( std::size_t vFrom, std::size_t vTo ){
	vvert_iterator it;
	if( !this->init_iterator( it, vFrom ) )
		return NULL;
	
	do{
		if( this->deref_iterator( it ) == vTo )
			return const_cast<half_edge*>( it.m_cur ); // Gross. This is just laziness.
	}while( this->advance_iterator( it ) );

	return NULL;
}

half_edge* EditMesh::find_edge( std::size_t vFrom, std::size_t vTo ){
	if( half_edge* he = this->find_twin( vFrom, vTo ) )
		return &m_heData[ he->twin ];
	return NULL;
}

bool EditMesh::flip_edge( half_edge &he ){
    half_edge &twin = m_heData[ he.twin ];

    if (he.face == HOLE_INDEX ||
        twin.face == HOLE_INDEX)
        return false;

    std::size_t he_tri[3];
    std::size_t twin_tri[3];

    // prep: gather half edge indices in
    // the order they should be after flip
    he_tri[0]   = he.next;
    twin_tri[0] = twin.next;
    he_tri[1]   = twin.twin;
    twin_tri[1] = he.twin;
    he_tri[2]   = m_heData[ twin_tri[0] ].next;
    twin_tri[2] = m_heData[ he_tri[0] ].next;

	if( m_heData[ he_tri[2] ].vert == m_heData[ twin_tri[2] ].vert )
		return false;

    // step 1: ensure he's verts don't point to
    // either half_edge (does not break mesh)
    m_vertData[ he.vert ] = twin_tri[0];
    m_vertData[ twin.vert ] = he_tri[0];

    // step 2: set the he's vert to new originating vert
    he.vert = m_heData[ twin_tri[2] ].vert;
    twin.vert = m_heData[ he_tri[2] ].vert;
    
    // step 3: ensure the faces point to one
    // of the half edges connected to them
    m_faceData[ he.face ] = he_tri[0];
    m_faceData[ twin.face ] = twin_tri[0];

    // step 4: fix two edges that will point
    // to the wrong face
    m_heData[he_tri[2]].face = he.face;
    m_heData[twin_tri[2]].face = twin.face;

    // step 5: ensure half edges point to
    // each other
    for( int i=0; i<3; ++i ) {
        m_heData[ he_tri[i] ].next = he_tri[(i+1)%3];
        m_heData[ twin_tri[i] ].next = twin_tri[(i+1)%3];
    }

    return true;
}

// IMPORTANT: Given a collection of half-edges to delete (ex. When removing a face we need to kill 2, 4, or 6 half-edges) they must be deleting in decreasing index order!
void EditMesh::delete_half_edge_impl( std::size_t he ){
	assert( (m_heData[he].vert >= m_vertData.size() || m_vertData[m_heData[he].vert] != he) && "Deleting this half_edge leaves a dangling link from a vertex. Must handle this first" );
		
	// Move a half_edge from the end overtop of the half_edge we are deleting, then update the indices of linked half_edges.
	m_heData[he] = m_heData.back();
	m_heData.pop_back();

	// We may have just deleted the item at the end of the list, so we have nothing to update since the indices didn't change.
	if( he != m_heData.size() ){
		const half_edge& heMoved = m_heData[he];

		// If the moved half_edge was the arbitrary half_edge linked to the vertex, update it.
		if( m_vertData[heMoved.vert] == m_heData.size() )
			m_vertData[heMoved.vert] = he;

		// If the moved half_edge was the arbitrary half_edge linked to the face, update it.
		if( heMoved.face != HOLE_INDEX && m_faceData[heMoved.face] == m_heData.size() )
			m_faceData[heMoved.face] = he;

		assert( heMoved.twin < m_heData.size() );
		assert( m_heData[heMoved.twin].twin == m_heData.size() );
		m_heData[heMoved.twin].twin = he;

		// NOTE: If we are deleting a bundle of half_edges, then by definition we must call delete_half_edge() in decreasing order of indices. That prevents
		//       me from having to worry about moving a partially destroyed half_edge into the 'he' position.

#ifdef USE_PREV
		assert( m_heData[heMoved.prev].next == m_heData.size() );
		m_heData[heMoved.prev].next = he;

		assert( m_heData[heMoved.next].prev == m_heData.size() );
		m_heData[heMoved.next].prev = he;
#else
		// Have to loop around the face until we find the half_edge using 'heMoved' as its 'next' entry, then update it.
		std::size_t hePrev = heMoved.next;
		while( m_heData[hePrev].next != m_heData.size() )
			hePrev = m_heData[hePrev].next;

		assert( m_heData[hePrev].next == m_heData.size() );
		m_heData[hePrev].next = he;
#endif
	}

	// Update the links in the simplification queue too.
	if( !m_simplifyQueue.empty() ){
		m_simplifyQueue[he] = m_simplifyQueue.back();
		m_simplifyQueue.pop_back();
	}
}

template <std::size_t N>
void EditMesh::delete_half_edges_impl( std::size_t (&heToDelete)[N] ){
	std::sort( heToDelete, heToDelete + N, std::greater<std::size_t>() );
	for( std::size_t i = 0; i < N; ++i )
		this->delete_half_edge_impl( heToDelete[i] );
}

bool g_debug = false;

std::size_t EditMesh::collapse_edge( std::size_t he ){
	assert( he < m_heData.size() );
	assert( m_heData[he].face != HOLE_INDEX && m_heData[m_heData[he].twin].face != HOLE_INDEX && "Cannot collapse a boundary edge" );

	const half_edge& heBase = m_heData[he];
	const half_edge& heTwin = m_heData[heBase.twin];

	// We are going to delete the faces on either side of the chosen edge, so we need to delete 3 half_edges and patch up the twin links on the 4
	// bordering edges.
	std::size_t heBorder[4];
	heBorder[0] = m_heData[ heBase.next ].twin;
	heBorder[1] = m_heData[ m_heData[ heBase.next ].next ].twin;
	heBorder[2] = m_heData[ m_heData[ heTwin.next ].next ].twin;
	heBorder[3] = m_heData[ heTwin.next ].twin;

	// TODO: Relax this assertion. We should be able to collapse a spike jutting into a hole.
	assert( ( m_heData[ heBorder[0] ].face != HOLE_INDEX || m_heData[ heBorder[1] ].face != HOLE_INDEX ) && "Cannot collapse an edge on a face with holes on either side." );
	assert( ( m_heData[ heBorder[2] ].face != HOLE_INDEX || m_heData[ heBorder[3] ].face != HOLE_INDEX ) && "Cannot collapse an edge on a face with holes on either side." );

	// Check if we can actually collapse. This checks for a degree 3 vertex at the vertices not on the edge we are collapsing.
	if( m_heData[ m_heData[ m_heData[ heBorder[1] ].next ].twin ].next == heBorder[0] )
		return HOLE_INDEX;
	if( m_heData[ m_heData[ m_heData[ heBorder[2] ].next ].twin ].next == heBorder[3] )
		return HOLE_INDEX;

	// Capture the indices of things (2 faces & 6 half-edges) we want to delete.
	std::size_t fToDelete[] = { heBase.face, heTwin.face };
	std::size_t heToDelete[] = { he, heBase.next, m_heData[ heBase.next ].next, heBase.twin, heTwin.next, m_heData[ heTwin.next ].next };
	
#ifndef NDEBUG
	// We can't be deleting border edges!
	for( auto i : heToDelete ){
		if( std::find( heBorder, heBorder + 4, i ) != heBorder + 4 )
			return HOLE_INDEX;	
		//assert( std::find( heBorder, heBorder + 4, i ) == heBorder + 4 );
	}

	if( g_debug ){
		std::vector< std::set<std::size_t> > verts( 3 );

		verts[0].insert( heBase.vert );
		verts[0].insert( heTwin.vert );

		for( int i = 1; i < verts.size(); ++i ){
			for( auto v : verts[i-1] ){
				vvert_iterator it;
				this->init_iterator( it, v );
				do{
					verts[i].insert( this->deref_iterator( it ) );
				}while( this->advance_iterator( it ) );
			}
		}

		std::vector<std::size_t> orderedVerts( verts.back().begin(), verts.back().end() );
		std::set<std::size_t> faces;

		std::vector< double > vpos;
		std::vector< std::size_t > finds;

		for( auto v : orderedVerts ){
			vpos.push_back( m_vertices[v].x() ); vpos.push_back( m_vertices[v].y() ); vpos.push_back( m_vertices[v].z() );
			//std::clog << "m.add_vert( " << m_vertices[v].x() << ", " << m_vertices[v].y() << ", " << m_vertices[v].z() << " );" << std::endl;
		}

		// Visit the 1-ring
		for( auto v : verts[1] ){
			vface_iterator it;
			this->init_iterator( it, v );
			do{
				if( this->deref_iterator( it ) != HOLE_INDEX && faces.find( this->deref_iterator( it ) ) == faces.end() ){
					faces.insert( this->deref_iterator( it ) );

					fvert_iterator itFace;
					this->init_iterator( itFace, this->deref_iterator( it ) );

					std::size_t f[3];
					std::size_t i = 0;
					do{
						f[i++] = std::find( orderedVerts.begin(), orderedVerts.end(), this->deref_iterator( itFace ) ) - orderedVerts.begin();
					}while( this->advance_iterator( itFace ) );

					finds.push_back( f[0] ); finds.push_back( f[1] ); finds.push_back( f[2] );
					//std::clog << "m.add_face( " << f[0] << ", " << f[1] << ", " << f[2] << " );" << std::endl;
				}	
			}while( this->advance_iterator( it ) );
		}

		std::size_t base = std::find( orderedVerts.begin(), orderedVerts.end(), heBase.vert ) - orderedVerts.begin();
		std::size_t twin = std::find( orderedVerts.begin(), orderedVerts.end(), heTwin.vert ) - orderedVerts.begin();
		std::clog << "m.collapse_edge( " << base << ", " << twin << " );" << std::endl;

		EditMesh m;
		m.init( vpos, finds );
		std::ofstream fout( "debug.obj" );
		m.write_to_obj_stream( fout );
		fout.close();
	}
#endif

	// We may also need to fix the vertex->half_edge link for the verts using these faces. There are technically 4, but we only update the 3 that are not going to be deleted.
	std::size_t verts[] = { this->prev( heBase ).vert, heBase.vert, this->prev( heTwin ).vert };

	// Move the base vertex (arbitrarily) to the middle of the edge. Could leave it where it is, or do something fancier too.
	m_vertices[heBase.vert] = 0.5 * ( m_vertices[heBase.vert] + m_vertices[heTwin.vert] ); 

	// Adjust all the twin's 1-ring to link to the vertex we are not going to delete.
	std::size_t heIt = this->twin(this->next(heBase)).next;
	std::size_t heEnd = heBase.twin;
	for( ; heIt != heEnd; heIt = this->twin( m_heData[heIt] ).next ){
		assert( m_heData[heIt].vert == heTwin.vert );
		
		// Associate to the other vertex now, so we can delete this one.
		m_heData[heIt].vert = heBase.vert;
	}

	// Fix the vert associations if required, picking a non-hole face.
	if( m_vertData[ verts[0] ] == m_heData[ heBorder[1] ].twin )
		m_vertData[ verts[0] ] = (m_heData[ heBorder[0] ].face != HOLE_INDEX) ? heBorder[0] : m_heData[ heBorder[1] ].next;
	if( m_vertData[ verts[1] ] == he || m_vertData[ verts[1] ] == heTwin.next )
		m_vertData[ verts[1] ] = (m_heData[ heBorder[1] ].face != HOLE_INDEX) ? heBorder[1] : heBorder[2];
	if( m_vertData[ verts[2] ] == m_heData[ heBorder[2] ].twin )
		m_vertData[ verts[2] ] = (m_heData[ heBorder[3] ].face != HOLE_INDEX) ? heBorder[3] : m_heData[ heBorder[2] ].next;

	// "Delete" the other vertex
	m_vertData[heTwin.vert] = HOLE_INDEX;

	// Collapse the two triangles bordering our chosen half-edge by connecting the opposite edges together.
	m_heData[ heBorder[0] ].twin = heBorder[1];
	m_heData[ heBorder[1] ].twin = heBorder[0];
	m_heData[ heBorder[2] ].twin = heBorder[3];
	m_heData[ heBorder[3] ].twin = heBorder[2];

	// Have to delete the faces in the proper order.
	if( fToDelete[0] < fToDelete[1] )
		std::swap( fToDelete[0], fToDelete[1] );

	this->delete_half_edges_impl( heToDelete );
	detail::delete_face( m_faceData, m_heData, fToDelete[0] );
	detail::delete_face( m_faceData, m_heData, fToDelete[1] );

	return verts[1];
}

std::size_t EditMesh::add_face( std::size_t (&f)[3] ){
	std::size_t faceIndex = m_faceData.size();
	std::size_t heIndex = m_heData.size();

	// Find the half-edges on the hole face we are filling. We must either:
	//  1. Find no half-edges, if all vertices are unconnected from the mesh.
	//  3. Find one half-edge, if one of the vertices is not connected to the existing mesh.
	//  4. Find two half-edges, if we are adding a triangle inside of a polygonal hole.
	//  2. Find three half-edges, if we are filling an existing triangular hole.
	half_edge* he[] = { 
		this->find_edge( f[0], f[1] ), 
		this->find_edge( f[1], f[2] ), 
		this->find_edge( f[2], f[0] ) };
	
	// Find the first half-edge we need to modify. This is an edge 
	std::size_t base = HOLE_INDEX;
	for( std::size_t i = 0; i < 3 && base == HOLE_INDEX; ++i ){
		if( he[i] ){
			assert( he[i]->face == HOLE_INDEX && "Non-manifold mesh detected. Cannot connect to an edge which already has two incident faces (ie. One side must be a hole)" );
			if( !he[(i+2)%3] )
				base = i;
		}
	}

	if( base == HOLE_INDEX ){
		// This triangle is not connected to any others, or we completely filled a triangular hole.
		if( he[0] /*|| he[1] || he[2]*/ ){
			assert( he[0] && he[1] && he[2] );
			assert( he[0]->face == HOLE_INDEX && he[1]->face == HOLE_INDEX && he[2]->face == HOLE_INDEX );
			assert( &m_heData[ he[0]->next ] == he[1] && &m_heData[ he[1]->next ] == he[2] && &m_heData[ he[2]->next ] == he[0] );
			
			// Update the face index of the triangular hole to convert it to a face.
			he[0]->face = he[1]->face = he[2]->face = faceIndex;
			m_faceData.push_back( he[2]->next );
		}else{
			assert( !he[0] && !he[1] && !he[2] );
			assert( m_vertData[f[0]] == HOLE_INDEX && m_vertData[f[1]] == HOLE_INDEX && m_vertData[f[2]] == HOLE_INDEX && "Non-manifold mesh detected. Cannot have two hole faces incident on a vertex." );

			// Make 3 new half-edges for the triangle, and 3 new half-edges for the hole outside of the triangle.
			half_edge newHe[6];
			detail::init( newHe[0], heIndex+1, heIndex+5, f[0], faceIndex );
			detail::init( newHe[1], heIndex+2, heIndex+4, f[1], faceIndex );
			detail::init( newHe[2], heIndex  , heIndex+3, f[2], faceIndex );
			detail::init( newHe[3], heIndex+4, heIndex+2, f[0], HOLE_INDEX );
			detail::init( newHe[4], heIndex+5, heIndex+1, f[2], HOLE_INDEX );
			detail::init( newHe[5], heIndex+3, heIndex  , f[1], HOLE_INDEX );
#ifdef USE_PREV
			newHe[0].prev = heIndex+2;
			newHe[1].prev = heIndex;
			newHe[2].prev = heIndex+1;

			newHe[3].prev = heIndex+5;
			newHe[4].prev = heIndex+3;
			newHe[5].prev = heIndex+4;
#endif

			m_vertData[ f[0] ] = heIndex;
			m_vertData[ f[1] ] = heIndex+1;
			m_vertData[ f[2] ] = heIndex+2;

			m_faceData.push_back( heIndex );
			m_heData.push_back( newHe[0] );
			m_heData.push_back( newHe[1] );
			m_heData.push_back( newHe[2] );
			m_heData.push_back( newHe[3] );
			m_heData.push_back( newHe[4] );
			m_heData.push_back( newHe[5] );
		}
	}else{
		std::size_t next = (base+1)%3, prev = (base+2)%3;
		std::size_t baseIndex = static_cast<std::size_t>( he[base] - &m_heData.front() );

		assert( !he[prev] );

		if( he[next] ){
			// We have two edges to steal from the hole, and we need to add two new half-edges
			half_edge newHe[2];
			detail::init( newHe[0], baseIndex, heIndex+1, f[prev], faceIndex );
			detail::init( newHe[1], he[next]->next, heIndex, f[base], HOLE_INDEX );

#ifdef USE_PREV
			newHe[0].prev = he[base]->next;
			newHe[1].prev = he[base]->prev;

			m_heData[ he[base]->prev ].next = heIndex + 1;
			m_heData[ he[next]->next ].prev = heIndex + 1;

			he[next]->next = heIndex;
			he[base]->prev = heIndex;
#else
			// Have to find the previous half_edge in the polygonal hole so we can point it to the new half-edge in the hole.
			half_edge* hePrev = &m_heData[ he[next]->next ];
			while( &m_heData[hePrev->next] != he[base] ){
				hePrev = &m_heData[hePrev->next];
				assert( hePrev != he[next] ); // To catch weirdness.
			}
			assert( &m_heData[hePrev->next] == he[base] );
			
			hePrev->next = heIndex + 1;
			he[next]->next = heIndex;
#endif

			// Update the face indices of the half-edges to indicate they are in a triangle now.
			he[base]->face = he[next]->face = faceIndex;

			m_faceData.push_back( heIndex );
			m_heData.push_back( newHe[0] );
			m_heData.push_back( newHe[1] );
		}else{
			assert( m_vertData[ f[prev] ] == HOLE_INDEX && "Non-manifold mesh detected. Cannot have two hole faces incident on a vertex." );

			// We have one edge to steal from the hole, and we need to add four new half-edges.
			half_edge newHe[4];
			detail::init( newHe[0], baseIndex, heIndex+2, f[prev], faceIndex );
			detail::init( newHe[1], heIndex  , heIndex+3, f[next], faceIndex );
			detail::init( newHe[2], heIndex+3, heIndex  , f[base], HOLE_INDEX );
			detail::init( newHe[3], he[base]->next, heIndex+1, f[prev], HOLE_INDEX );

#ifdef USE_PREV
			newHe[0].prev = heIndex+1;
			newHe[1].prev = baseIndex;
			newHe[2].prev = he[base]->prev;
			newHe[3].prev = heIndex+2;

			m_heData[ he[base]->prev ].next = heIndex+2;
			m_heData[ he[base]->next ].prev = heIndex+3;

			he[base]->prev = heIndex;
			he[base]->next = heIndex+1;
#else
			// Have to find the previous half_edge in the polyognal hole so we can point it to the new half-edge in the hole.
			half_edge* hePrev = &m_heData[ he[base]->next ];
			while( &m_heData[hePrev->next] != he[base] ){
				hePrev = &m_heData[hePrev->next];
				assert( hePrev != he[next] ); // To catch weirdness.
			}
			assert( &m_heData[hePrev->next] == he[base] );
			
			hePrev->next = heIndex+2;
			he[base]->next = heIndex+1;
#endif

			// Update the face indices of the half-edges to indicate they are in a triangle now.
			he[base]->face = faceIndex;

			m_vertData[f[prev]] = heIndex;
			m_faceData.push_back( heIndex );
			m_heData.push_back( newHe[0] );
			m_heData.push_back( newHe[1] );
			m_heData.push_back( newHe[2] );
			m_heData.push_back( newHe[3] );
		}
	}

	return faceIndex;
}

void EditMesh::delete_face( std::size_t f ){
	assert( f < m_faceData.size() );

	// We can assume that this face has 3 half-edges.
	std::size_t heIndices[3];
	heIndices[0] = m_faceData[f];
	
	half_edge* he[3];
	he[0] = &m_heData[heIndices[0]];
	he[1] = &m_heData[he[0]->next];
	he[2] = &m_heData[he[1]->next];

	heIndices[1] = he[0]->next;
	heIndices[2] = he[1]->next;
	
	assert( he[0]->face == f && he[1]->face == f && he[2]->face == f );
	assert( he[2]->next == m_faceData[f] );

	// Search for an edge that has a neighbor, but its prev edge doesn't. This is a canonical place to construct the algorithm from.
	std::size_t base = HOLE_INDEX;
	for( std::size_t i = 0; i < 3 && base == HOLE_INDEX; ++i ){
		if( m_heData[he[i]->twin].face != HOLE_INDEX && m_heData[he[(i+2)%3]->twin].face == HOLE_INDEX )
			base = i;
	}

	if( base == HOLE_INDEX ){
		if( m_heData[he[0]->twin].face == HOLE_INDEX ){
			// This is a lone triangle, so delete its half-edges and the exterior hole surrounding it too.

			// TODO: Remove the floating vertices? Currently we are leaving them.
			m_vertData[he[0]->vert] = HOLE_INDEX;
			m_vertData[he[1]->vert] = HOLE_INDEX;
			m_vertData[he[2]->vert] = HOLE_INDEX;

			// Delete all of the edges (both inside & outside half-edges). Must do this last since indices can change arbitrarily when deleting.
			std::size_t toDelete[] = { 
				heIndices[0], heIndices[1], heIndices[2], 
				he[0]->twin, he[1]->twin, he[2]->twin 
			};
			
			this->delete_half_edges_impl( toDelete );
			detail::delete_face( m_faceData, m_heData, f );
		}else{
			// This is an interior triangle. Only have to change the face_index to HOLE_INDEX for these edges.

			// Adjust any vertex references to new edges in non-hole faces.
			if( m_vertData[he[0]->vert] == heIndices[0] )
				m_vertData[he[0]->vert] = he[2]->twin;
			if( m_vertData[he[1]->vert] == heIndices[1] )
				m_vertData[he[1]->vert] = he[0]->twin;
			if( m_vertData[he[2]->vert] == heIndices[2] )
				m_vertData[he[2]->vert] = he[1]->twin;

			// Flag all these half-edges as being a hole now.
			he[0]->face = he[1]->face = he[2]->face = HOLE_INDEX;
			detail::delete_face( m_faceData, m_heData, f );
		}
	}else{
		std::rotate( he, he+base, he+3 );
		std::rotate( heIndices, heIndices+base, heIndices+3 );
		assert( m_heData[he[0]->twin].face != HOLE_INDEX );
		assert( m_heData[he[2]->twin].face == HOLE_INDEX );

		if( m_heData[he[1]->twin].face != HOLE_INDEX ){
			// We have one edge to remove, and a hole to connect to.
#ifdef USE_PREV
			he[1]->next = m_heData[he[2]->twin].next;
			he[0]->prev = m_heData[he[2]->twin].prev;
			m_heData[he[1]->next].prev = heIndices[1];
			m_heData[he[0]->prev].next = heIndices[0];
#else
			he[1]->next = m_heData[he[2]->twin].next;

			std::size_t hePrev = he[1]->next;
			while( m_heData[hePrev].next != he[2]->twin )
				hePrev = m_heData[hePrev].next;

			assert( m_heData[hePrev].next == he[2]->twin );
			m_heData[hePrev].next = heIndices[0];
#endif

			assert( m_heData[ m_vertData[ he[0]->vert ] ].face != HOLE_INDEX );
			assert( m_heData[ m_vertData[ he[1]->vert ] ].face != HOLE_INDEX );
			assert( m_heData[ m_vertData[ he[2]->vert ] ].face != HOLE_INDEX );

			// We may need to update the vertices if they referenced the edges we are deleting. Choose new half-edges that are inside 
			// non-hole triangles.
			if( m_vertData[he[0]->vert] == heIndices[0] )
				m_vertData[he[0]->vert] = m_heData[he[0]->twin].next;
			if( m_vertData[he[1]->vert] == heIndices[1] )
				m_vertData[he[1]->vert] = he[0]->twin;
			if( m_vertData[he[2]->vert] == heIndices[2] )
				m_vertData[he[2]->vert] = he[1]->twin;

			assert( m_heData[ m_vertData[ he[0]->vert ] ].face != HOLE_INDEX );
			assert( m_heData[ m_vertData[ he[1]->vert ] ].face != HOLE_INDEX );
			assert( m_heData[ m_vertData[ he[2]->vert ] ].face != HOLE_INDEX );

			he[0]->face = he[1]->face = HOLE_INDEX;

			std::size_t toDelete[] = { heIndices[2], he[2]->twin };

			// Delete the edges and face. Must do this last since indices can change arbitrarily when deleting.
			this->delete_half_edges_impl( toDelete );
			detail::delete_face( m_faceData, m_heData, f );
		}else{
			// We have two edges to remove, a vertex that will become floating, and a hole to connect to.
#ifdef USE_PREV
			he[0]->next = m_heData[he[1]->twin].next;
			he[0]->prev = m_heData[he[2]->twin].prev;
			m_heData[he[0]->next].prev = heIndices[0];
			m_heData[he[0]->prev].next = heIndices[0];
#else
			he[0]->next = m_heData[he[1]->twin].next;

			std::size_t hePrev = he[0]->next;
			while( m_heData[hePrev].next != he[2]->twin )
				hePrev = m_heData[hePrev].next;

			assert( m_heData[hePrev].next == he[2]->twin );
			m_heData[hePrev].next = heIndices[0];
#endif

			// We may need to update the vertices if they referenced the edges we are deleting. Choose new half-edges that are inside 
			// non-hole triangles.
			if( m_vertData[he[1]->vert] == heIndices[1] )
				m_vertData[he[1]->vert] = he[0]->twin;
			if( m_vertData[he[0]->vert] == he[2]->twin || m_vertData[he[0]->vert] == heIndices[0] )
				m_vertData[he[0]->vert] = m_heData[he[0]->twin].next;
			m_vertData[he[2]->vert] = HOLE_INDEX;

			// Update the face association of the one half-edge we are keeping (it joins the hole).
			he[0]->face = HOLE_INDEX;
			
			// Delete the edges and face. Must do this last since indices can change arbitrarily when deleting.
			std::size_t toDelete[] = { 
				heIndices[1], heIndices[2], 
				he[1]->twin, he[2]->twin 
			};

			this->delete_half_edges_impl( toDelete );
			detail::delete_face( m_faceData, m_heData, f );
		}
	}
}

std::size_t EditMesh::split_face_center( std::size_t f, std::size_t (*pOutFaceIndices)[3] ){
	assert( f < m_faceData.size() );
	assert( m_faceData[f] < m_heData.size() && m_heData[m_faceData[f]].face == f );

	std::size_t he[3];
	he[0] = m_faceData[f];
	he[1] = m_heData[he[0]].next;
	he[2] = m_heData[he[1]].next;

	assert( m_heData[he[2]].next == he[0] );
	assert( m_heData[he[0]].vert < m_vertices.size() && m_heData[he[1]].vert < m_vertices.size() && m_heData[he[2]].vert < m_vertices.size() );
	
	// New vert at face center
	Eigen::Vector3d newVert = ( m_vertices[ m_heData[ he[0] ].vert ] + m_vertices[ m_heData[ he[1] ].vert ] + m_vertices[ m_heData[ he[2] ].vert ] ) / 3.0;

	std::size_t newVertIndex = m_vertices.size();
	m_vertices.push_back( newVert );

	// Each half-edge gets associated to a new face, and we add 6 half-edges from the old vertices to the new.
	std::size_t newHeIndex = m_heData.size();
	std::size_t newFaceIndex = m_faceData.size();

	if( pOutFaceIndices ){
		(*pOutFaceIndices)[0] = f;
		(*pOutFaceIndices)[1] = newFaceIndex;
		(*pOutFaceIndices)[2] = newFaceIndex+1;
	}

	// Create six new half-edges connecting the center vertex to the old triangle corners.
	half_edge newHe[6];
	detail::init( newHe[0], newHeIndex+1, newHeIndex+3, m_heData[he[1]].vert, f );
	detail::init( newHe[1], he[0]       , newHeIndex+4, newVertIndex        , f );
	detail::init( newHe[2], newHeIndex+3, newHeIndex+5, m_heData[he[2]].vert, newFaceIndex );
	detail::init( newHe[3], he[1]       , newHeIndex  , newVertIndex        , newFaceIndex );
	detail::init( newHe[4], newHeIndex+5, newHeIndex+1, m_heData[he[0]].vert, newFaceIndex+1 );
	detail::init( newHe[5], he[2]       , newHeIndex+2, newVertIndex        , newFaceIndex+1 );

	// Connect the old half-edges to the new ones, and update their face association.
	//m_heData[he[0]].face = f;
	m_heData[he[0]].next = newHeIndex;
	m_heData[he[1]].face = newFaceIndex;
	m_heData[he[1]].next = newHeIndex+2;
	m_heData[he[2]].face = newFaceIndex+1;
	m_heData[he[2]].next = newHeIndex+4;

#ifdef USE_PREV
	newHe[0].prev = he[0];
	newHe[1].prev = newHeIndex;
	newHe[2].prev = he[1];
	newHe[3].prev = newHeIndex+2;
	newHe[4].prev = he[2];
	newHe[5].prev = newHeIndex+4;

	m_heData[he[0]].prev = newHeIndex+1;
	m_heData[he[1]].prev = newHeIndex+3;
	m_heData[he[2]].prev = newHeIndex+5;
#endif

	m_vertData.push_back( newHeIndex+3 ); // Arbitrary from 1, 3 & 5
	m_faceData[f] = he[0];
	m_faceData.push_back( he[1] );
	m_faceData.push_back( he[2] );
	m_heData.push_back( newHe[0] );
	m_heData.push_back( newHe[1] );
	m_heData.push_back( newHe[2] );
	m_heData.push_back( newHe[3] );
	m_heData.push_back( newHe[4] );
	m_heData.push_back( newHe[5] );

	return newVertIndex;
}

void EditMesh::split_boundary_edge( std::size_t heToSplit, std::size_t (*pOutVertIndices)[2], std::size_t (*pOutFaceIndices)[3] ){
	assert( heToSplit < m_heData.size() );
	assert( m_heData[heToSplit].face != HOLE_INDEX && m_heData[m_heData[heToSplit].twin].face == HOLE_INDEX );

	half_edge& heBase = m_heData[heToSplit];
	half_edge& heNext = m_heData[heBase.next];
	half_edge& hePrev = m_heData[heNext.next];
	half_edge& heTwin = m_heData[heBase.twin];

	Eigen::Vector3d newVert1 = m_vertices[ heBase.vert ] + ( m_vertices[ heNext.vert ] - m_vertices[ heBase.vert ] ) / 3.0;
	Eigen::Vector3d newVert2 = m_vertices[ heBase.vert ] + ( m_vertices[ heNext.vert ] - m_vertices[ heBase.vert ] ) * (2.0 / 3.0);

	// Construct 2 new faces and 8 new half_edges connecting the new verts to the off-edge vert.
	std::size_t newVertIndex = m_vertices.size();
	std::size_t newHeIndex = m_heData.size();
	std::size_t newFaceIndex = m_faceData.size();

	if( pOutVertIndices ){
		(*pOutVertIndices)[0] = newVertIndex;
		(*pOutVertIndices)[1] = newVertIndex+1;
	}

	if( pOutFaceIndices ){
		(*pOutFaceIndices)[0] = heBase.face;
		(*pOutFaceIndices)[1] = newFaceIndex;
		(*pOutFaceIndices)[2] = newFaceIndex+1;
	}

	half_edge newHe[8];
	detail::init( newHe[0], heNext.next , newHeIndex+1, newVertIndex   , heBase.face );
	detail::init( newHe[1], newHeIndex+4, newHeIndex  , hePrev.vert    , newFaceIndex );
	detail::init( newHe[2], newHeIndex+1, newHeIndex+3, newVertIndex+1 , newFaceIndex );
	detail::init( newHe[3], newHeIndex+5, newHeIndex+2, hePrev.vert    , newFaceIndex+1 );
	detail::init( newHe[4], newHeIndex+2, newHeIndex+6, newVertIndex   , newFaceIndex );
	detail::init( newHe[5], heBase.next , heBase.twin , newVertIndex+1 , newFaceIndex+1 );
	detail::init( newHe[6], newHeIndex+7, newHeIndex+4, newVertIndex+1 , HOLE_INDEX );
	detail::init( newHe[7], heTwin.next , heToSplit   , newVertIndex   , HOLE_INDEX );

#ifdef USE_PREV
	newHe[0].prev = heToSplit;
	newHe[1].prev = newHeIndex+2;
	newHe[2].prev = newHeIndex+4;
	newHe[3].prev = heBase.next;
	newHe[4].prev = newHeIndex+1;
	newHe[5].prev = newHeIndex+3;
	newHe[6].prev = heBase.twin;
	newHe[7].prev = newHeIndex+6;

	heNext.prev = newHeIndex+5;
	m_heData[heTwin.next].prev = newHeIndex+7
#endif

	heBase.next = newHeIndex;
	heBase.twin = newHeIndex+7;
	heTwin.next = newHeIndex+6;
	heTwin.twin = newHeIndex+5;
	heNext.next = newHeIndex+3;
	heNext.face = newFaceIndex+1;

	m_vertices.push_back( newVert1 );
	m_vertices.push_back( newVert2 );
	m_vertData.push_back( newHeIndex+4 );
	m_vertData.push_back( newHeIndex+5 );
	m_faceData[heBase.face] = heToSplit;
	m_faceData.push_back( newHeIndex+4 );
	m_faceData.push_back( newHeIndex+5 );
	m_heData.push_back( newHe[0] );
	m_heData.push_back( newHe[1] );
	m_heData.push_back( newHe[2] );
	m_heData.push_back( newHe[3] );
	m_heData.push_back( newHe[4] );
	m_heData.push_back( newHe[5] );
	m_heData.push_back( newHe[6] );
	m_heData.push_back( newHe[7] );
}

double EditMesh::get_cotan_weight( const vvert_iterator& it ) const {
	const half_edge *itCur = it.m_cur;
	const half_edge *itTwin = &m_heData[ itCur->twin ];

	double result = 0;

	Eigen::Vector3d a = this->get_vertex( itTwin->vert );
	Eigen::Vector3d b = this->get_vertex( itCur->vert );

	assert( ( itCur->face != HOLE_INDEX || itTwin->face != HOLE_INDEX ) && "Invalid mesh: edge with no face on either side" );

	if( itCur->face != HOLE_INDEX ){
		Eigen::Vector3d c = this->get_vertex( this->prev( *itCur ).vert );
		Eigen::Vector3d e0 = b - c;
		Eigen::Vector3d e1 = a - c;

		// We use the dot product and norm of the cross product to get cos and sin respectively. cotan = cos / sin
		result += static_cast<double>( e0.dot(e1) ) / e0.cross(e1).norm();
	}

	if( itTwin->face != HOLE_INDEX ){
		Eigen::Vector3d c = this->get_vertex( this->prev( *itTwin ).vert );
		Eigen::Vector3d e0 = a - c;
		Eigen::Vector3d e1 = b - c;

		result += static_cast<double>( e0.dot(e1) ) / e0.cross(e1).norm();
	}
	
	return result;
}

double EditMesh::get_mean_value_weight( const vvert_iterator& it ) const {
	const half_edge *itCur = it.m_cur;
	const half_edge *itTwin = &m_heData[ itCur->twin ];

	double result = 0;

	Eigen::Vector3d a = this->get_vertex( itTwin->vert );
	Eigen::Vector3d b = this->get_vertex( itCur->vert );
	
	Eigen::Vector3d e0 = (b - a);
	double eLen = e0.norm();

	e0 /= eLen;

	assert( ( itCur->face != HOLE_INDEX || itTwin->face != HOLE_INDEX ) && "Invalid mesh: edge with no face on either side" );

	if( itCur->face != HOLE_INDEX ){
		Eigen::Vector3d c = this->get_vertex( this->prev( *itCur ).vert );
		Eigen::Vector3d e1 = (c - a).normalized();

		result += std::tan( 0.5 * std::acos( e0.dot(e1) ) );
	}

	if( itTwin->face != HOLE_INDEX ){
		Eigen::Vector3d c = this->get_vertex( this->prev( *itTwin ).vert );
		Eigen::Vector3d e1 = (c - a).normalized();

		result += std::tan( 0.5 * std::acos( e0.dot(e1) ) );
	}
	
	return result / eLen;
}

Eigen::Vector3d EditMesh::get_normal( const vface_iterator& it ) const {
	Eigen::Vector3d a = this->get_vertex( it.m_next->vert );
	Eigen::Vector3d b = this->get_vertex( it.m_cur->vert );
	Eigen::Vector3d c = this->get_vertex( m_heData[ it.m_cur->next ].vert );
	
	return ( a - c ).cross( b - c ).normalized();
}

Eigen::Vector4d EditMesh::get_plane( const vface_iterator& it ) const {
	Eigen::Vector3d a = this->get_vertex( it.m_next->vert );
	Eigen::Vector3d b = this->get_vertex( it.m_cur->vert );
	Eigen::Vector3d c = this->get_vertex( m_heData[ it.m_cur->next ].vert );
	Eigen::Vector3d n = ( a - c ).cross( b - c ).normalized();

	// Plane equation Ax + By + Cz + D = 0 -> n.dot( [x,y,z] ) - n.dot( c ) = 0
	return Eigen::Vector4d( n.x(), n.y(), n.z(), -n.dot( c ) );
}

Eigen::Matrix4d get_quadric_error_matrix( EditMesh& m, std::size_t vertex ){
	Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
	
	vface_iterator it;
	if( m.init_iterator( it, vertex ) ){
		do{
			// Skip holes. There can be only one per 1-ring or else this mesh is non-manifold.
			if( m.deref_iterator( it ) != HOLE_INDEX ){
				Eigen::Vector4d p = m.get_plane( it );
				result = ( result + p * p.transpose() ); // Construct the 4x4 Rank-1 distance matrix via outer product and add it in.
			}
		}while( m.advance_iterator( it ) );
	}

	return result;
}

double get_quadric_error( EditMesh& m, std::size_t v1, std::size_t v2 ){
	Eigen::Matrix4d errorMatrix = get_quadric_error_matrix( m, v1 ) + get_quadric_error_matrix( m, v2 );
	Eigen::Matrix4d derivMatrix = errorMatrix;
	derivMatrix.row( 3 ) = Eigen::Vector4d( 0, 0, 0, 1 );

	Eigen::FullPivLU< Eigen::Matrix4d > lu( derivMatrix );
	if( lu.isInvertible() ){
		// Find the point which minimizes the square error.
		Eigen::Vector4d	hp = lu.solve( Eigen::Vector4d(0,0,0,1) );
		return hp.dot( errorMatrix * hp );
	}else{
		// Use the edge midpoint.
		Eigen::Vector3d p = 0.5 * ( m.get_vertex( v1 ) + m.get_vertex( v2 ) );
		Eigen::Vector4d	hp( p.x(), p.y(), p.z(), 1.0 );
		return hp.dot( errorMatrix * hp );
	}
}

void collapse_one_edge( EditMesh& m ){
	if( m.m_simplifyData.size() != m.m_vertData.size() ){
		for( std::size_t i = m.m_simplifyData.size(), iEnd = m.get_vert_size(); i < iEnd; ++i )
			m.m_simplifyData.push_back( get_quadric_error_matrix( m, i ) );
		
		m.m_simplifyQueue.resize( m.m_heData.size() );

		for( std::size_t i = 0, iEnd = m.m_heData.size(); i < iEnd; ++i ){
			// Only store data for one of the half_edges.
			if( i > m.m_heData[i].twin ){
				m.m_simplifyQueue[i].value = m.m_simplifyQueue[ m.m_heData[i].twin ].value;
			}else{
				std::size_t v1 = m.m_heData[ i ].vert;
				std::size_t v2 = m.m_heData[ m.m_heData[i].twin ].vert;

				m.m_simplifyQueue[ i ].value = get_quadric_error( m, v1, v2 );
			}
		}
	}

	Eigen::Matrix4d errorMatrix;
	std::size_t v;

	do{
		double bestValue = std::numeric_limits<double>::max();
		std::size_t bestIndex = HOLE_INDEX;

		// Find the lowest error half_edge. This *could* be optimized with a priority queue or heap, but that's freaking complicated ...
		for( std::size_t i = 0, iEnd = m.m_heData.size(); i < iEnd; ++i ){
			if( m.m_simplifyQueue[i].value < bestValue ){
				bestValue = m.m_simplifyQueue[i].value;
				bestIndex = i;
			}
		}

		if( bestIndex == HOLE_INDEX ){
			// We couldn't collapse anything.
			std::cerr << "Can't collapse anything!" << std::endl;
			return;
		}

		// Store the new error matrix while our vert indices are still valid.
		errorMatrix = m.m_simplifyData[ m.m_heData[bestIndex].vert ] + m.m_simplifyData[ m.m_heData[ m.m_heData[bestIndex].twin ].vert ];

		v = m.collapse_edge( bestIndex );

		if( v == HOLE_INDEX )
			m.m_simplifyQueue[ bestIndex ].value = std::numeric_limits<double>::quiet_NaN();

	}while( v == HOLE_INDEX );

	m.m_simplifyData[v] = errorMatrix;

	// Update the error values for all the verts in the new 1-ring.
	vvert_iterator it;
	if( m.init_iterator( it, v ) ){
		do{
			std::size_t he = m.deref_iterator_left_edge( it );
			std::size_t heTwin = m.deref_iterator_right_edge( it );

			m.m_simplifyQueue[ he ].value = m.m_simplifyQueue[ heTwin ].value = get_quadric_error( m, v, m.deref_iterator( it ) );
		}while( m.advance_iterator( it ) );
	}

	m.flag_editted();
}

Eigen::Vector3d EditMesh::get_vnormal( std::size_t i ) const {
	const half_edge *itCur = &m_heData[ m_heData[ m_vertData[ i ] ].twin ];
	const half_edge *itEnd = itCur;

    Eigen::Vector3d normal;
    normal.Zero();

    Eigen::Vector3d center = this->get_vertex( i );
    Eigen::Vector3d vec_prev;
    Eigen::Vector3d vec_curr = this->get_vertex( this->prev( m_heData[ itCur->twin ] ).vert ) - center;

    do {
        vec_prev = vec_curr;
        vec_curr = this->get_vertex( itCur->vert ) - center;

        normal += vec_curr.cross(vec_prev).normalized();
        itCur = &this->twin( this->next( *itCur ) );
    } while (itCur != itEnd);

    return normal.normalized();
}

Eigen::Vector3d EditMesh::get_fnormal( std::size_t i ) const {
	const half_edge *e1 = &m_heData[ m_faceData[ i ] ];
    const half_edge *e2 = &this->next(*e1);
    const half_edge *e3 = &this->next(*e2);

    Eigen::Vector3d a = this->get_vertex( e1->vert );
    Eigen::Vector3d b = this->get_vertex( e2->vert );
    Eigen::Vector3d c = this->get_vertex( e3->vert );

    Eigen::Vector3d normal = (b - a).cross(c - a);

    return normal.normalized();
}

void EditMesh::subdivide_sqrt3( std::size_t num_steps ) {
    /* NOTE: I do things in a wonky order in order to preserve all the
     *       information I need within the mesh
     * 1) Create new point for each face
     * 2) move old points prior to adding new points to mesh
     * 3) assign new faces using the newly made points
     * 4) flip edges
     *
     * NOTE2: we know that sqrt3 method only works on triangle meshes, so we can 
     * bail if the mesh is non-triangular
     */

    // allow the filter to iterate multiple times with one call
    for( int s = num_steps; s > 0; s-- ) {

        int mesh_size = this->m_faceData.size();
        int vert_size = this->m_vertData.size(); // to be used for step 3
        int border_count = 0;

        // update per mesh attribute to keep track of the
        // whether the current iteration is even or odd
        // to be used for border conditions
        bool isEven = ((++subdiv_iter) % 2) == 0;

        /* NOTE: this relies on the fact that all new faces are allocated at the end of the
         * chunk of memory given (which must be true so that an iterator over new elements will work)
         * Knowing this, we can iterate over all elements of the initial mesh by starting at element
         * zero and only incrementing the size of the original mesh.*/

        /*******************************************
         * STEP 1: Find the Positions for Old Verts
         *******************************************/
        std::vector<Eigen::Vector3d> new_loc(vert_size);

        for( int i = 0; i < vert_size; i++ ) {
            std::size_t vert_index = m_vertData[i];
            vvert_iterator vit;

            // if floating vertex, ignore
            if( !this->init_iterator(vit, i) )
                continue;

            Eigen::Vector3d        tmp_val  = Eigen::Vector3d::Zero();
            unsigned int           valence  = 0;
            float                  alpha    = 0;

            // for this subdivision scheme, boundary verts are treated differently
            // the original boundary vert will have a weighted new location of
            // 1/27 (4*(p_{i-1} + p_{i+1}) + 19*p_i)
            // the new boundary verts will have other weights as shown later
            if( this->reset_boundary_iterator(vit) ) {
                border_count++; // note this vertex as a border vertex
                tmp_val += this->get_vertex(vit.m_cur->vert);

                this->advance_iterator(vit);
                tmp_val += this->get_vertex(vit.m_cur->vert);

                new_loc[i] = (this->get_vertex(i) * 19 + tmp_val * 4)/27;
            }
            else {
                do {
                    tmp_val += this->get_vertex(vit.m_cur->vert);
                    valence++;
                } while( this->advance_iterator(vit) );

                alpha = (4 - 2 * cos( 2 * M_PI / valence)) / 9;
                new_loc[i] = this->get_vertex(i) * (1 - alpha) + tmp_val * (alpha / valence);
            }
        }

        /*******************************************
         * STEP 2: Assign New Faces
         *******************************************/
        /* NOTE: boundary faces are subdivided differently
         * on even and odd iterations */

        // NOTE: need to iterate backwards so that deletion does cause our
        // counter to be wrong. This works because we use a swap and pop 
        // mechanism upon deletion, and thus is probably not the best approach
        for( int i = mesh_size-1; i >= 0; --i ) {
            std::size_t face_index = m_faceData[i];
            std::size_t old_verts[3];

            // TODO: verify that this should never be a hole?

            // if a boundary, and even iteration we subdivide differently
            if( isEven && this->isBoundaryFace(i)) {
                Eigen::Vector3d  v_value = Eigen::Vector3d::Zero();
                half_edge       *he_cur = &m_heData[ face_index ];
                std::size_t      he_id = NULL;

                // the one-ring of boundary verts around face's
                // boundary edge
                Eigen::Vector3d  ring_verts[4];

                // progress half-edge to boundary
                while( m_heData[ he_cur->twin ].face != HOLE_INDEX )
                    he_cur = &m_heData[ he_cur->next ];

                // progress to vert furthest to the right
                // denoted X
                //
                //    ^   ^   ^
                //  V___V___V___X
                he_cur = &m_heData[ he_cur->next ];
                while( m_heData[ he_cur->twin ].face != HOLE_INDEX )
                    he_cur = &m_heData[ m_heData[ he_cur->twin ].next ];
                he_cur = &m_heData[ he_cur->twin ];

                ring_verts[0] = this->get_vertex( he_cur->vert );
                he_cur = &m_heData[ he_cur->next ];
                he_id = he_cur->twin;

                assert( m_heData[he_id].face == i && "Now you've fucked up" );

                ring_verts[1] = this->get_vertex( he_cur->vert );
                he_cur = &m_heData[ he_cur->next ];
                ring_verts[2] = this->get_vertex( he_cur->vert );
                he_cur = &m_heData[ he_cur->next ];
                ring_verts[3] = this->get_vertex( he_cur->vert );

                std::size_t vid[2];
                this->split_boundary_edge(he_id, &vid);

                // calculate the positions for the new verts
                this->set_vertex(vid[0], (16*ring_verts[1] + 10*ring_verts[2] + ring_verts[0])/27 );
                this->set_vertex(vid[1], (16*ring_verts[2] + 10*ring_verts[1] + ring_verts[3])/27 );
            }
            else {
                // calculate new vertex position
                std::size_t      vert_index = 0;
                half_edge       *he_face    = &m_heData[ face_index ];
                Eigen::Vector3d  v_value    = Eigen::Vector3d::Zero();

                for( int j = 0; j < 3; ++j ) {
                    old_verts[j] = he_face->vert;
                    v_value += this->get_vertex(old_verts[j]);
                    he_face = &m_heData[ he_face->next ];
                }
                v_value /= 3;

                vert_index = this->split_face_center(i);
                this->set_vertex(vert_index, v_value);
            }
        }

        /*******************************************
         * STEP 3: Move Old Vertices
         *******************************************/
        /* NOTE: we account for the original mesh
         * having loose vertices with j */
        for( unsigned int i = 0; i < vert_size; i++ ) {
            m_vertices[i] = new_loc[i];
        }

        /*******************************************
         * STEP 4: Flip Edges
         *******************************************/
        for( int i = 0, s = m_heData.size(); i < s; i++ ){
            half_edge *main = &m_heData[i];
            half_edge *twin = &m_heData[main->twin];
            if( i < main->twin && // remove duplicate calls (once per edge, not half edge)
                main->vert < vert_size && twin->vert < vert_size && // must be old edge (both sides on old verts)
                main->face != HOLE_INDEX && twin->face != HOLE_INDEX ) // must not be boundary edge
                this->flip_edge(*main);
                
        }
    } // end for ( number of iterations )

    // This has to be incremented for the rendered mesh to update!!!
    edit_count++;
}

bool EditMesh::remesh_flatten(std::size_t vert_id,
                std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &v_ring,
                std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &v_ring_flat) {
    vvert_iterator vit;
    if( !this->init_iterator(vit, vert_id) )
		return false;
    
    // if on boundary, do not move
    if (this->reset_boundary_iterator(vit))
        return false;

    // gather the positions of the one ring
    // in CCW order
    // center vertex is at index 0
    Eigen::Vector3d orig_p = this->get_vertex( vert_id );
    do {
        v_ring.push_back( this->get_vertex(vit.m_cur->vert));
    } while (this->advance_iterator(vit));

    // calculate the original edge vectors
    std::size_t valence = v_ring.size();
    std::vector<Eigen::Vector3d> orig_edge(valence);
    for( int i = 0; i < valence; i++ )
        orig_edge[i] = ( v_ring[i] - orig_p );

    // calculate the original angles (to the left of vector)
    double sum = 0;
    std::vector<double> orig_a(valence, 0);
    for( int i = 0; i < valence; i++ ) {
        Eigen::Vector3d v1 = orig_edge[i].normalized();
        Eigen::Vector3d v2 = orig_edge[(i+1)%valence].normalized();
        orig_a[i] = acos( std::min( std::max( v1.dot( v2 ), -1.0 ), 1.0 ) ); // returns angle in rad
		//assert( orig_a[i] == orig_a[i] );

        sum += orig_a[i]; // total sum of angles
    }

    if (sum < 1e-8 || sum != sum)
        return false;

    Eigen::Vector2d displace(1,0);

    // calculate the new 2d positions
    // by rotating a unit vector by each of the 2pi normalized angles
    for( int i = 0; i < valence; i++) {
		assert( std::abs( displace.norm() - 1.0 ) < 1e-8 );
        v_ring_flat.push_back( orig_edge[i].norm() * displace );
        displace = Eigen::Rotation2Dd(orig_a[i] * 2 * M_PI / sum ) * displace;
    }
    return true;
}

Eigen::Vector3d EditMesh::remesh_unflatten(Eigen::Vector3d &orig_p, Eigen::Vector2d &pt_2d,
                    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &v_ring,
                    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &flatv) {
    double v, w, u;
    int flat_size = flatv.size();
    for( int i = 0; i < flat_size; i++ ) {
        Eigen::Vector2d v0 = flatv[i];
        Eigen::Vector2d v1 = flatv[(i+1)%flat_size];

        // Compute barycentric coordinates (u, v, w)
        double d00 = v0.dot(v0);
        double d01 = v0.dot(v1);
        double d11 = v1.dot(v1);
        double d20 = pt_2d.dot(v0);
        double d21 = pt_2d.dot(v1);
        double denom = d00 * d11 - d01 * d01;
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;

        // if inside triangle
        if( v >= -1e-8 && w >= -1e-8 && v<=1+1e-8 && w<=1+1e-8 ) {
            u = 1.0f - v - w;
            return u * orig_p + v * v_ring[i] + w * v_ring[(i+1)%flat_size];
        }
    }

    //assert(false && "The new vertex should always fall within the one ring, so you should never get here");
    return orig_p;
}

Eigen::Vector3d EditMesh::remesh_relocateVertDelaunay(std::size_t vert_id) {
    
    Eigen::Vector3d orig_p = this->get_vertex( vert_id );
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > orig_ring;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > flatv;

    // if on boundary, do not move
    if (!this->remesh_flatten(vert_id, orig_ring, flatv))
        return orig_p;

    // perform delaunay edge flips until you have a proper triangulation
    for( int i = flatv.size() - 1; i >= 0; --i ) {
        Eigen::Vector2d center = flatv[i] / 2;
        double rad = center.norm();

        // if the vert to either side is not delaunay,
        // then the edge flip would remove this vertex from
        // the one ring
        int size = flatv.size();
        if( (flatv[(i+1)%size] - center).norm() <= rad ||
            (flatv[(i+(size-1))%size] - center).norm() <= rad ) {
                flatv.erase(flatv.begin() + i);
                orig_ring.erase(orig_ring.begin() + i);
        }
    }

    // 4 times: (practical convergence)
    // for each vert in one ring
    //     find delaunay center (circumcetner?)
    // average centers
    Eigen::Vector2d pt_2d = Eigen::Vector2d::Zero();
    int flat_size = flatv.size();
    for( int i = 0; i < 4; ++i ) {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > av(flat_size, Eigen::Vector2d::Zero());
        for( int j = 0; j < flat_size; ++j )
            av[j] = (pt_2d + flatv[j] + flatv[(j+1)%flat_size])/3;

        pt_2d = Eigen::Vector2d::Zero();
        for( int j = 0; j < flat_size; ++j )
            pt_2d += av[j];
        pt_2d /= flat_size;
    }

    return this->remesh_unflatten(orig_p, pt_2d, orig_ring, flatv);
}

Eigen::Vector3d EditMesh::remesh_relocateVert(std::size_t vert_id) {
    Eigen::Vector3d orig_p = this->get_vertex( vert_id );
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > orig_ring;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > flatv;

    // if on boundary, do not move
    if (!this->remesh_flatten(vert_id, orig_ring, flatv))
        return orig_p;

    int valence = orig_ring.size();
    // sum weighted new positions
    //  lv cv rv
    //   ._._.
    //   \ | /
    //     .
    //     pv
    Eigen::Vector2d ci = Eigen::Vector2d::Zero();
    double sumsqr_alpha = 0;
    for( int i = 0; i < valence; i++ ) {
        Eigen::Vector2d r = flatv[(i+(valence-1)) % valence];
        Eigen::Vector2d l = flatv[(i+1) % valence];
        Eigen::Vector2d c = flatv[i];
        double d = c.norm();
        l = (l - c).normalized();
        r = (r - c).normalized();
        c = c / d;

        double alphai = (acos(c.dot(l)) + acos(r.dot(c))) / 2; // alpha i
        sumsqr_alpha += 1/(alphai*alphai);
        ci += 1/(alphai*alphai) * ((d * (Eigen::Rotation2Dd(alphai) * l)) + flatv[i]);
    }

    // calculate the final position
    Eigen::Vector2d pt_2d = (1/sumsqr_alpha) * ci;
    
    return this->remesh_unflatten(orig_p, pt_2d, orig_ring, flatv);
}

Eigen::Vector3d EditMesh::remesh_areaBased(std::size_t vert_id) {
    Eigen::Vector3d orig_p = this->get_vertex( vert_id );
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > orig_ring;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > flatv;

    // if on boundary, do not move
    if (!this->remesh_flatten(vert_id, orig_ring, flatv))
        return orig_p;

    // construct matrices to solve
    double area_total = 0;
    int    valence    = orig_ring.size();
    Eigen::Vector2d *pi, *pii;
    Eigen::MatrixX2d A(valence, 2);
    Eigen::VectorXd b(valence);

    for( int i = 0; i < valence; ++i ) {
        pi  = &flatv[i];
        pii = &flatv[(i+1)%valence];
        A(i, 0) = ((*pi)[1] - (*pii)[1])/2; // (1/2)(y_{i} + y_{i+1}) x
        A(i, 1) = ((*pii)[0] - (*pi)[0])/2; // -(1/2)(x_{i+1} - x_{i}) y

        b(i) = ( ((*pi)[1] * (*pii)[0]) - ((*pi)[0] * (*pii)[1]) )/2;
        area_total -= b(i); // actual area added to sum (subtracted to cancel out above minus)
    }

    // complete b with weighted value of total area.
    // for our weights we are using the average
    for( int i = 0; i < valence; ++i )
        b(i) += area_total/valence;

    // solve system for final positions
    Eigen::PartialPivLU<Eigen::Matrix2d> solver(A.transpose()*A);
    Eigen::Vector2d pt_2d = solver.solve(A.transpose() * b);

    Eigen::Vector3d temp =  this->remesh_unflatten(orig_p, pt_2d, orig_ring, flatv);
	return temp;
}

void EditMesh::remesh_delaunayize() {

    // perform delaunay edge flips until there are no more avaliable
    int size = m_heData.size();
    for(int i = 0; i < size; i++) {
        // get half-edge i, both end points and both opposite verts
        half_edge *cur  = &m_heData[i];
        half_edge *twin = &m_heData[ cur->twin ];

        // already visited this edge (on other half)
        if (i < cur->twin)
            continue;

        cur  = &m_heData[ cur->next ];
        twin = &m_heData[ twin->next ];

        Eigen::Vector3d v[4];
        v[0] = this->get_vertex( twin->vert ); // one side of edge
        v[1] = this->get_vertex( cur->vert ); // other side of edge
        v[2] = this->get_vertex( m_heData[twin->next].vert ); // opposite cw
        v[3] = this->get_vertex( m_heData[cur->next].vert ); // opposite ccw

        Eigen::Vector3d center1 = (v[0]+v[1])/2;
        double          radius1 = ((v[1]-v[0])).norm()/2 - 1e-7;

        Eigen::Vector3d center2 = (v[2]+v[3])/2;
        double          radius2 = ((v[3]-v[2])).norm()/2 + 1e-7;

        double l1 = (v[2]-center1).norm();
        double l2 = (v[3]-center1).norm();
        double l3 = (v[0]-center2).norm();
        double l4 = (v[1]-center2).norm();

        if( (l1 < radius1 || l2 < radius1) &&
            (l3 > radius2 && l4 > radius2) ) {
                this->flip_edge(m_heData[i]);
        }
    }

    // This has to be incremented for the rendered mesh to update!!!
    edit_count++;
}

void EditMesh::remesh_relocate(std::size_t num_steps) {
    for( int iter = 0; iter < num_steps; ++iter ) {
        int vsize = m_vertData.size();
        std::vector<Eigen::Vector3d> new_loc(m_vertices.size());
        for( int i = 0; i < vsize; ++i )
            new_loc[i] = this->remesh_relocateVert(i);
        for( int i = 0; i < vsize; ++i )
            this->set_vertex(i, new_loc[i]);
    }

    // This has to be incremented for the rendered mesh to update!!!
    edit_count++;
}

void EditMesh::remesh_area(std::size_t num_steps) {
    for( int iter = 0; iter < num_steps; ++iter ) {
        int vsize = m_vertData.size();
        std::vector<Eigen::Vector3d> new_loc(m_vertices.size());
        for( int i = 0; i < vsize; ++i )
            new_loc[i] = this->remesh_areaBased(i);
        for( int i = 0; i < vsize; ++i )
            this->set_vertex(i, new_loc[i]);
    }

    // This has to be incremented for the rendered mesh to update!!!
    edit_count++;
}

void EditMesh::remesh() {
    for( int j = 0; j < 4; ++j )
        remesh_delaunayize();
    for( int i = 0; i < 4; ++i) {
        remesh_area();
        for( int j = 0; j < 4; ++j )
            remesh_delaunayize();
        remesh_relocate();
        for( int j = 0; j < 4; ++j )
            remesh_delaunayize();        
    }
}

void EditMesh::remesh_pretty() {
    remesh();
    subdivide_sqrt3();
    for( int j = 0; j < 4; ++j )
        remesh_delaunayize();
    remesh_area();
}

void EditMesh::get_draw_data( float *verts, int *indices ) const {

    /* get each vertex only once. This is good for efficiency
     * but results in bad looking meshes due to each vertex 
     * having a fixed normal

        for( std::size_t i = 0, iEnd = m_faceData.size(); i < iEnd; i++ ){
            const half_edge* he = &m_heData[ m_faceData[i] ];

            for( int j = 0; j < 3; j++){
                indices[3*i +j] = he->vert;
                he = &this->next(*he);
            }
        }

        for( std::size_t i = 0, iEnd = m_vertData.size(); i < iEnd; i++ ){
		    Eigen::Vector3d vert = this->get_vertex( i );
            for( int j = 0; j < 3; j++)
                verts[3*i+j] = (float) vert[j];
        }
    */

    // for each face
    for( std::size_t i = 0, iEnd = m_faceData.size(); i < iEnd; i++ ){
        const half_edge* he = &m_heData[ m_faceData[i] ];

        // for each vertex of the face
        for( int j = 0; j < 3; j++){
            Eigen::Vector3d vert = this->get_vertex(he->vert);
            indices[3*i+j] = 3*i+j;

            // for each component of the vertex
            for( int k = 0; k < 3; k++){
                verts[3*(3*i+j) + k] = vert[k];
            }
            he = &this->next(*he);
        }
    }
}

void EditMesh::get_draw_normals( float *normals ) const {

    /* this finds the averaged vertex normals which results in
     * poor looking meshes when they are not smooth

        for( std::size_t i = 0, iEnd = m_vertData.size(); i < iEnd; i++ ){
		    Eigen::Vector3d normal = this->get_normal( i );
            for( int j = 0; j < 3; j++)
                normals[3*i+j] = (float) normal[j];
        }
    */

    for( std::size_t i = 0, iEnd = m_faceData.size(); i < iEnd; i++ ){
		Eigen::Vector3d normal = this->get_fnormal( i );

        for( int j = 0; j < 3; j++){
            for( int k = 0; k < 3; k++){
                normals[3*(3*i+j) + k] = normal[k];
            }
        }
    }
}

// call instead of init to test edge flip
// easiest way is hacking it into mesh constructor
void EditMesh::test_flip() {
    std::vector<double> xyz;
    std::vector<std::size_t> faces;

    // four verts
    xyz.push_back(-1); xyz.push_back(0); xyz.push_back(0);
    xyz.push_back(0); xyz.push_back(1); xyz.push_back(1);
    xyz.push_back(0); xyz.push_back(1); xyz.push_back(-1);
    xyz.push_back(1); xyz.push_back(0); xyz.push_back(0);

    // two triangles
    faces.push_back(0); faces.push_back(1);
    faces.push_back(2); faces.push_back(2);
    faces.push_back(1); faces.push_back(3);

    this->init(xyz, faces);

    half_edge *he = &m_heData[0];
    for( int i = 0; i < m_heData.size(); i++ ) {
        he = &m_heData[i];
        if (he->face != HOLE_INDEX &&
            m_heData[ he->twin ].face != HOLE_INDEX )
            break;
    }
    flip_edge(*he);
    this->edit_count++;
}

void EditMesh::write_to_obj_stream( std::ostream& stream ) const {
	for( auto& v : m_vertices )
		stream << "v " << v.x() << ' ' << v.y() << ' ' << v.z() << std::endl;
	stream << std::endl;
	for( std::size_t i = 0, iEnd = m_faceData.size(); i < iEnd; ++i ){
		fvert_iterator it;
		this->init_iterator( it, i );
		stream << "f ";
		bool isFirst = true;
		do{
			if( !isFirst )
				stream << ' ';
			isFirst = false;
			stream << this->deref_iterator( it )+1;
		}while( this->advance_iterator( it ) );
		stream << std::endl;
	}
}

void EditMesh::verify() const {
	for( std::size_t i = 0, iEnd = m_faceData.size(); i < iEnd; ++i ){
		std::size_t c = 0;
		
		const half_edge* it = &m_heData[ m_faceData[i] ];
		assert( it->next != m_faceData[i] );
		while( it->next != m_faceData[i] ){
			assert( it->face == i );
			assert( it->next != HOLE_INDEX && it->twin != HOLE_INDEX && it->vert < m_vertData.size() );
			assert( ( m_heData[ it->twin ].face == HOLE_INDEX || m_heData[ m_heData[it->next].twin ].face != m_heData[ it->twin ].face ) && "Can't have two edges shared between the same faces!" );
			it = &m_heData[it->next];
			assert( ++c < 1000000 ); // This isn't strictly a problem, but probably no face has a million verts in it.
		}
	}

	for( std::size_t i = 0, iEnd = m_vertData.size(); i < iEnd; ++i ){
		assert( m_vertData[i] == HOLE_INDEX || m_vertData[i] < m_heData.size() );
		if( m_vertData[i] != HOLE_INDEX ){
			const half_edge* it = &m_heData[ m_vertData[i] ];
			assert( it->vert == i );
			assert( it->face != HOLE_INDEX && "By convention, vertices should not reference hole faces" );
		}
	}

	for( std::size_t i = 0, iEnd = m_heData.size(); i < iEnd; ++i ){
		const half_edge* it = &m_heData[i];
		assert( it->vert < m_vertData.size() );
		assert( it->face == HOLE_INDEX || it->face < m_faceData.size() );

		assert( it->next < m_heData.size() );
		assert( it->next != i );
		assert( m_heData[it->next].face == it->face );
		assert( m_heData[it->next].vert != it->vert );

		assert( it->twin < m_heData.size() );
		assert( m_heData[it->twin].twin == i );
		assert( m_heData[it->twin].face != it->face );
		assert( m_heData[it->twin].vert == m_heData[it->next].vert );

#ifdef USE_PREV
		assert( it->prev < m_heData.size() );
		assert( it->prev != i );
		assert( m_heData[it->next].prev == i );
		assert( m_heData[it->prev].next == i );
		assert( m_heData[it->prev].face == it->face );
		assert( m_heData[it->prev].vert != it->vert );
#endif
	}
}

void EditMesh::test(){
	EditMesh m1, m2, m3;

	std::vector<double> v;
	std::vector<std::size_t> f;

	v.push_back( 0 ); v.push_back( 0 ); v.push_back( 0 );
	v.push_back( 1 ); v.push_back( 0 ); v.push_back( 0 );
	v.push_back( 0 ); v.push_back( 1 ); v.push_back( 0 );
	v.push_back( 1 ); v.push_back( 1 ); v.push_back( 0 );
	v.push_back( 2 ); v.push_back( 1 ); v.push_back( 0 );
	v.push_back( 1 ); v.push_back( 2 ); v.push_back( 0 );

	f.push_back( 0 ); f.push_back( 1 ); f.push_back( 2 );
	f.push_back( 2 ); f.push_back( 1 ); f.push_back( 3 );
	f.push_back( 3 ); f.push_back( 1 ); f.push_back( 4 );
	f.push_back( 4 ); f.push_back( 5 ); f.push_back( 3 );
	f.push_back( 3 ); f.push_back( 5 ); f.push_back( 2 );

	m1.init( v, f );

	for( std::size_t i = 0, iEnd = v.size(); i < iEnd; i += 3 )
		assert( m2.add_vertex( v[i], v[i+1], v[i+2] ) == i/3 );
	for( std::size_t i = 0, iEnd = f.size(); i < iEnd; i += 3 )
		assert( m2.add_face( *reinterpret_cast<std::size_t(*)[3]>( &f[i] ) ) == i/3 );

	assert( m1.get_face_size() == m2.get_face_size() );
	assert( m1.get_vert_size() == m2.get_vert_size() );

	m1.verify();
	m2.verify();

	m2.delete_face( 0 );
	m2.verify();

	m2.delete_face( 0 ); // Was face 4 originally
	m2.verify();

	m2.delete_face( 0 ); // Was face 3 originally
	m2.verify();

	m2.delete_face( 0 ); // Was face 2 originally
	m2.verify();

	m2.delete_face( 0 ); // Was face 1 originally
	m2.verify();

	assert( m2.get_face_size() == 0 );

	m2.add_face( 0, 1, 2 );
	m2.verify();

	m2.split_face_center( 0 );
	m2.verify();

	assert( m2.get_face_size() == 3 );

	m2.clear();
	m2.add_vertex( 0, 0, 0 );
	m2.add_vertex( 1, 0, 0 );
	m2.add_vertex( 0, 1, 0 );
	m2.add_face( 0, 1, 2 );
	m2.split_boundary_edge( m2.find_twin( 0, 1 )->twin );
	m2.verify();

	assert( m2.get_face_size() == 3 );

	m3.add_vertex( -1, 0, 0 );
	m3.add_vertex( 1, 0, 0 );
	m3.add_vertex( 0, 1, 0 );
	m3.add_vertex( 0, -1, 0 );
	m3.add_vertex( -1, 1, 0 );
	m3.add_vertex( -1, -1, 0 );
	m3.add_vertex( 1, 1, 0 );
	m3.add_vertex( 1, -1, 0 );

	m3.add_face( 0, 1, 2 );
	m3.add_face( 0, 2, 4 );
	m3.add_face( 0, 4, 5 );
	m3.add_face( 0, 5, 3 );
	m3.add_face( 0, 3, 1 );
	
	m3.add_face( 1, 3, 7 );
	m3.add_face( 1, 7, 6 );
	m3.add_face( 1, 6, 2 );
	m3.verify();

	m3.flip_edge( *m3.find_edge( 1, 0 ) );
	m3.verify();
	m3.flip_edge( *m3.find_edge( 2, 3 ) );
	m3.verify();

	std::size_t newVert = m3.collapse_edge( m3.find_edge( 1, 0 )->twin );
	assert( newVert != HOLE_INDEX );
	m3.verify();

	/*m2.clear();
	m2.add_vertex( 0.010744, 0.483695, 0.298761 );
	m2.add_vertex( 0.010538, 0.484281, 0.305409 );
	m2.add_vertex( 0.014906, 0.48369, 0.304997 );
	m2.add_vertex( 0.006473, 0.484811, 0.30548 );
	m2.add_vertex( 0.010333, 0.484867, 0.312038 );
	m2.add_vertex( 0.004998, 0.485704, 0.314376 );
	m2.add_vertex( 0.010129, 0.485783, 0.323883 );
	m2.add_vertex( 0.016209, 0.484307, 0.313866 );
	m2.add_face( 7, 1, 4 );
	m2.add_face( 7, 6, 1 );
	m2.add_face( 6, 3, 1 );
	m2.add_face( 5, 3, 6 );
	m2.add_face( 5, 0, 3 );
	m2.add_face( 3, 2, 4 );
	m2.add_face( 3, 0, 2 );
	m2.add_face( 1, 3, 4 );

	for( std::size_t i = m2.get_face_size(); i > 0; --i ){
		m2.delete_face( i-1 );
		m2.verify();
	}*/

	m1.clear();
	m1.add_vertex( -1, 0, 0 );
	m1.add_vertex( 1, 0, 0 );
	m1.add_vertex( 0, 1, 0 );
	m1.add_vertex( -2, 1, 0 );
	m1.add_vertex( -2, -1, 0 );
	m1.add_vertex( 0, -1, 0 );
	m1.add_vertex( 2, -1, 0 );
	m1.add_vertex( 2, 1, 0 );
	m1.add_vertex( -2, 2, 0 );
	m1.add_vertex( -3, 1, 0 );
	m1.add_vertex( -3, -1, 0 );
	m1.add_vertex( -2, -2, 0 );
	m1.add_vertex( 2, -2, 0 );
	m1.add_vertex( 3, -1, 0 );
	m1.add_vertex( 3, 1, 0 );
	m1.add_vertex( 2, 2, 0 );

	m1.add_face( 0, 1, 2 );
	m1.add_face( 0, 2, 3 );
	m1.add_face( 0, 3, 4 );
	m1.add_face( 0, 4, 5 );
	m1.add_face( 0, 5, 1 );
	m1.add_face( 1, 5, 6 );
	m1.add_face( 1, 6, 7 );
	m1.add_face( 1, 7, 2 );
	m1.add_face( 3, 2, 8 );
	m1.add_face( 3, 8, 9 );
	m1.add_face( 3, 9, 10 );
	m1.add_face( 3, 10, 4 );
	m1.add_face( 4, 10, 11 );
	m1.add_face( 4, 11, 5 );
	m1.add_face( 5, 11, 12 );
	m1.add_face( 5, 12, 6 );
	m1.add_face( 6, 12, 13 );
	m1.add_face( 6, 13, 14 );
	m1.add_face( 6, 14, 7 );
	m1.add_face( 7, 14, 15 );
	m1.add_face( 7, 15, 2 );
	m1.add_face( 2, 15, 8 );

	m1.verify();

	std::vector< std::size_t > faces;
	vface_iterator it;
	if( m1.init_iterator( it, 0 ) ){
		do{
			std::vector< std::size_t >::iterator itInsert = std::lower_bound( faces.begin(), faces.end(), m1.deref_iterator( it ), std::greater<std::size_t>() );
			if( itInsert == faces.end() || *itInsert != m1.deref_iterator( it ) )
				faces.insert( itInsert, m1.deref_iterator( it ) );
		}while( m1.advance_iterator( it ) );
	}
	if( m1.init_iterator( it, 1 ) ){
		do{
			std::vector< std::size_t >::iterator itInsert = std::lower_bound( faces.begin(), faces.end(), m1.deref_iterator( it ), std::greater<std::size_t>() );
			if( itInsert == faces.end() || *itInsert != m1.deref_iterator( it ) )
				faces.insert( itInsert, m1.deref_iterator( it ) );
		}while( m1.advance_iterator( it ) );
	}

	std::swap( faces[faces.size()-1], faces[faces.size()-2] );

	for( auto face : faces ){
		m1.delete_face( face );
		m1.verify();
	}

	std::unique_ptr<EditMesh> em( loadEditMeshFromFile("Mesh/Collapse-2Ring.obj") );
	em->verify();
	em->collapse_edge( 5, 4 );
	em->verify();

	/*em.reset( loadEditMeshFromFile("Mesh/camel.obj") );

	for( std::size_t i = 0; i < 9667; ++i ){
		collapse_one_edge( *em );
		if ( i > 9000 || i % 1000 == 0 )
			em->verify();
	}*/

	em->clear();
	em->add_vertex(2, .345, 0);
	em->add_vertex(4, 0, 0);
	em->add_vertex(-4, 0, 0);
	em->add_vertex(0, 1, 0);
	em->add_vertex(0, -1, 0);

	em->add_face(0, 3, 2);
	em->add_face(0, 2, 4);
	em->add_face(0, 4, 1);
	em->add_face(0, 1, 3);

	em->remesh_areaBased(0);
}

bool EditMesh::is_safe_addface( std::size_t v1, std::size_t v2, std::size_t v3 ) {

    std::set<std::size_t> vv;
    vv.insert( m_vertData[v1] );
    vv.insert( m_vertData[v2] );
    vv.insert( m_vertData[v3] );


    // if there's one disconnected vertex its safe
    // more than one is not
    if( vv.size() < 3 )
        return false;
    else if( vv.count(HOLE_INDEX) > 0 )
        return true;

    vv.clear();
    vv.insert(v1);
    vv.insert(v2);
    vv.insert(v3);

    int count = 0;
    vvert_iterator vit;
    init_iterator(vit, v1);
    do {
        if (vit.m_cur->vert == v2 ||
            vit.m_cur->vert == v3) {
            count++;
            break;
        }
    } while( this->advance_iterator(vit) );

    init_iterator(vit, v2);
    do {
        if (vit.m_cur->vert == v1 ||
            vit.m_cur->vert == v3) {
            count++;
            break;
        }
    } while( this->advance_iterator(vit) );

    // if two of the vertices have a next within the triplet, its safe
    if ( count > 1 )
        return true;

    // else unsafe
    return false;
    //return (v1 != HOLE_INDEX && v2 != HOLE_INDEX) ||
    //       (v2 != HOLE_INDEX && v3 != HOLE_INDEX) ||
    //       (v3 != HOLE_INDEX && v1 != HOLE_INDEX);
}