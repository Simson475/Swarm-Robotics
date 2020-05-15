#pragma once

#include <iostream>
#include <vector>
#include <list>
#include "GL/glut.h"
#include <utility>

//using namespace std;

namespace DS
{

    template<class G>
	class CEdge
	{
		public :

		typedef typename G::EdgeData E;
		typedef typename G::Node Node;

		E data;
		Node* nodes[2];
		bool bidir;
        bool isSaved=false;
        bool isDrawable=false;

		CEdge( E edge_data, bool edge_bidir )
		{
			data = edge_data;
			bidir = edge_bidir;
		}
	};

    

    template<class G>
	class CNode
	{
		public :

		typedef typename G::NodeData N;
		typedef typename G::Edge Edge;
		
        int id;
        bool isVisit=false;
        //bool isClose=false;
        //bool isOpen=false;
        bool isAlone=false;
        int idCluster;

		

		N data;
		vector<Edge*> edges;
		vector<Edge*> edges_back;

		CNode( N node_data, int _id )
		{
            id=_id;
			data = node_data;
		}
	};    



	template<class N, class E>
	class CGraph
	{
		public :

		typedef CGraph<N,E> self;
		typedef CNode<self> Node;
		typedef CEdge<self> Edge;

		typedef N NodeData;
		typedef E EdgeData;

		vector<Node*> nodes;

		Node* insertNode( N node_data, int id );
        Node* insertNode( N node_data, int id, int idCluster );
		void insertEdge( Node* from, Node* to, E edge_data, bool dir );
		void removeEdge( Edge* edge );
		void removeNode( Node* node );

		void draw();
        
        
        //info extra 
        int nEdges=0;
        vector<pair <int,int> > edgesLog;

		void print();
	};


	template<class N, class E>
	CNode<CGraph<N,E> >* CGraph<N,E>::insertNode( N node_data, int id)
	{
        CNode<CGraph<N,E> >* oNode=new CNode<CGraph<N,E> >( node_data, id );
		nodes.push_back(oNode);
        return oNode;
	}
    
    template<class N, class E>
	CNode<CGraph<N,E> >* CGraph<N,E>::insertNode( N node_data, int id, int idCluster)
	{
        CNode<CGraph<N,E> >* oNode=new CNode<CGraph<N,E> >( node_data, id );
        oNode->idCluster=idCluster;
		nodes.push_back(oNode);
        return oNode;
	}

	template<class N, class E>
	void CGraph<N,E>::insertEdge( CNode<CGraph<N,E> >* from,
								  CNode<CGraph<N,E> >* to,
								  E edge_data, bool bidir )
	{
		CEdge<CGraph<N,E> >* _edge_from = new CEdge<CGraph<N,E> >( edge_data, bidir );

		_edge_from->nodes[0] = from;
		_edge_from->nodes[1] = to;
        _edge_from->isDrawable=true;
        
		from->edges.push_back( _edge_from );
		to->edges_back.push_back( _edge_from );
        
		if ( bidir )
		{
			CEdge<CGraph<N,E> >* _edge_to = new CEdge<CGraph<N,E> >( edge_data, bidir );

			_edge_to->nodes[0] = to;
			_edge_to->nodes[1] = from;
			to->edges.push_back( _edge_to );
			from->edges_back.push_back( _edge_to );
		}
        edgesLog.push_back(make_pair (from->id,to->id));
        nEdges++;

	}

	template<class N, class E>
	void CGraph<N,E>::removeEdge( CEdge<CGraph<N,E> >* edge )
	{
		// Remove from the "from" node's edges
		CNode<CGraph<N,E> >* _node_from = edge->nodes[0];
		
		int _indx = -1;
		for ( int q = 0; q < _node_from->edges.size(); q++ )
		{
			if ( _node_from->edges[q] == edge )
			{
				_indx = q;
				break;
			}
		}
		
		if ( _indx != -1 )
		{
			_node_from->edges.erase( _node_from->edges.begin() + _indx );
		}

		// Remove from the "to" node's edges
		CNode<CGraph<N,E> >* _node_to = edge->nodes[1];
		
		_indx = -1;
		for ( int q = 0; q < _node_to->edges_back.size(); q++ )
		{
			if ( _node_to->edges_back[q] == edge )
			{
				_indx = q;
				break;
			}
		}
		
		if ( _indx != -1 )
		{
			_node_to->edges_back.erase( _node_to->edges_back.begin() + _indx );
		}

		delete edge;
	}

	template<class N, class E>
	void CGraph<N,E>::removeNode( CNode<CGraph<N,E> >* node )
	{
		// Remove edges from the "from" node and, if bidir, from the "to" this node
		vector<Edge*> _edges = node->edges;
		for ( int q = 0; q < _edges.size(); q++ )
		{
			removeEdge( _edges[q] );
		}

		vector<Edge*> _edges_back = node->edges_back;
		for ( int q = 0; q < _edges_back.size(); q++ )
		{
			removeEdge( _edges_back[q] );
		}

		// Remove the node
		int _indx = -1;
		for ( int q = 0; q < nodes.size(); q++ )
		{
			if ( nodes[q] == node )
			{
				_indx = q;
				break;
			}
		}

		if ( _indx != -1 )
		{
			nodes.erase( nodes.begin() + _indx );
		}
	}
}