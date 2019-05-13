/**
\file     graph.h
\author   Brennan Cathcart <brennancathcart@gmail.com>
*/
#ifndef GRAPH_H
#define GRAPH_H

#include <algorithm>
#include <iostream>
#include <map>
#include <queue>
#include <vector>
using namespace std;

// Classes
template <class T>
class Edge;
template <class T>
class Vertex;
template <class T>
class Graph;

/*** EDGE ***/

template <class T>
class Edge
{
public:
    Edge(const T dest, const double weight, const T* id = nullptr);
    void setWeight(const double weight);
    const double weight() const;
    const T dest() const;
    const T id() const;

private:
    T dest_;
    double weight_;
    T id_;  // Optional identifier
};

template <class T>
Edge<T>::Edge(const T dest, const double weight, const T* id)
    : dest_(dest),
      weight_(weight)
{
    if (id != nullptr)
        id_ = *id;
}

template <class T>
void Edge<T>::setWeight(const double weight)
{
    weight_ = weight;
}

template <class T>
const double Edge<T>::weight() const
{
    return weight_;
}

template <class T>
const T Edge<T>::dest() const
{
    return dest_;
}

template <class T>
const T Edge<T>::id() const
{
    return id_;
}

/*** VERTEX ***/

template <class T>
class Vertex
{
public:
    ~Vertex();
    bool addEdge(const T dest, const double weight, const T* id = nullptr);
    bool updateEdgeWeight(const T dest, const double weight) const;
    bool removeEdge(const T dest);
    const double weight(const T dest) const;
    const vector<Edge<T>* > edgeList() const;

private:
    vector<Edge<T>* > edges_;

    Edge<T>* getEdge(const T dest) const;
};

template <class T>
Vertex<T>::~Vertex()
{
    for (const Edge<T>* edge : edges_)
    {
        delete edge;
    }
}

template <class T>
bool Vertex<T>::addEdge(const T dest, const double weight, const T* id)
{
    if (!getEdge(dest))
    {
        edges_.push_back(new Edge<T>(dest, weight, id));
        return true;
    }
    else
    {
        return false;
    }
}

template <class T>
bool Vertex<T>::updateEdgeWeight(const T dest, const double weight) const
{
    Edge<T>* edge = getEdge(dest);
    if (edge)
    {
        edge->setWeight(weight);
        return true;
    } 
    else
    {
        return false;
    }
}

template <class T>
bool Vertex<T>::removeEdge(const T dest)
{
    for (class vector<Edge<T>* >::iterator it = edges_.begin(); it != edges_.end(); it++)
    {
        if ((*it)->dest() == dest)
        {
            edges_.erase(it);
            return true;
        }
    }
    return false;
}

template <class T>
const double Vertex<T>::weight(const T dest) const
{
    const Edge<T>* edge = getEdge(dest);
    if (edge)
    {
        return edge->weight();
    } 
    else
    {
        return std::numeric_limits<double>::infinity();
    }
}

template <class T>
const vector<Edge<T>* > Vertex<T>::edgeList() const
{
    return edges_;
}

template <class T>
Edge<T>* Vertex<T>::getEdge(const T dest) const
{   
    for (Edge<T>* edge : edges_)
    {
        if (edge->dest() == dest)
        {
            return edge;
        }
    }
    return nullptr;
}

/*** GRAPH ***/

template <class T>
class Graph
{
public:
    Graph();
    Graph(bool use_edge_id);
    ~Graph();

    bool addVertex(const T id);
    bool removeVertex(const T id);
    bool addEdge(const T src, const T dest, const double weight, const T* id = nullptr);
    bool removeEdge(const T src, const T dest);
    bool updateEdgeWeight(const T src, const T dest, const double weight);
    const vector<T> bestPath(const T src, const T dest) const;
    void print() const;

private:
    map<const T, Vertex<T>* > vertices_;
    bool use_edge_id_;
};

template <class T>
Graph<T>::Graph()
    : use_edge_id_(false) {}

template <class T>
Graph<T>::Graph(bool use_edge_id)
    : use_edge_id_(use_edge_id) {}

template <class T>
Graph<T>::~Graph()
{
    for (auto const &pair : vertices_)
    {
        delete pair.second;
    }
}

template <class T>
bool Graph<T>::addVertex(const T id)
{
    if (vertices_.count(id))
    {
        cerr << "warning: cannot add vertex: vertex already exists" << endl;
        return false;
    }

    vertices_[id] = new Vertex<T>();
    return true;
}

template <class T>
bool Graph<T>::removeVertex(const T id)
{
    class map<const T, Vertex<T>* >::iterator vertex_it = vertices_.find(id);
    if (vertex_it == vertices_.end())
    {
        cerr << "warning: cannot remove vertex: vertex doesn't exists" << endl;
        return false;
    }

    Vertex<T>* vertex = vertex_it->second;
    vertices_.erase(vertex_it);

    // Remove all incoming edges
    for (auto const &pair : vertices_)
    {
        pair.second->removeEdge(id);
    }

    delete vertex;
    return true;
}

template <class T>
bool Graph<T>::addEdge(const T src, const T dest, const double weight, const T* id)
{
    Vertex<T>* src_vertex = nullptr;
    Vertex<T>* dest_vertex = nullptr;

    if (src == dest)
    {
        cerr << "error: cannot add edge: source and destination vertices are the same." << endl;
        return false;
    }

    if (use_edge_id_ && id == nullptr)
    {
        cerr << "error: cannot add edge: edge ID expected." << endl;
        return false;
    }

    class map<const T, Vertex<T>* >::iterator src_it = vertices_.find(src);

    // Add vertices if they don't exist
    if (src_it == vertices_.end())
    {
        cout << "note: source vertex doesn't exist: adding " << src << endl;
        this->addVertex(src);
        src_it = vertices_.find(src);
    }
    if (!vertices_.count(dest))
    {
        cout << "note: destination vertex doesn't exist: adding " << dest << endl;
        this->addVertex(dest);
    }

    if (!src_it->second->addEdge(dest, weight, id))
    {
        cerr << "warning: cannot add edge: edge already exists" << endl;
        return false;
    }

    return true;
}

template <class T>
bool Graph<T>::updateEdgeWeight(const T src, const T dest, const double weight)
{
    Vertex<T>* src_vertex = nullptr;
    Vertex<T>* dest_vertex = nullptr;

    if (src == dest)
    {
        cerr << "warning: cannot update edge wight: source and destination vertices are the same" << endl;
        return false;
    }

    class map<const T, Vertex<T>* >::iterator src_it = vertices_.find(src);
    if (src_it == vertices_.end())
    {
        cerr << "warning: cannot update edge weight: source vertex doesn't exist" << endl;
    }
    if (!vertices_.count(dest))
    {
        cerr << "warning: cannot update edge weight: destination vertex doesn't exist" << endl;
    }

    if (!src_it->second->updateEdgeWeight(dest, weight))
    {
        cerr << "warning: cannot update edge weight: edge doesn't exist" << endl;
        return false;
    }

    return true;
}

template <class T>
bool Graph<T>::removeEdge(const T src, const T dest)
{
    class map<const T, Vertex<T>* >::iterator src_it = vertices_.find(src);

    if (src_it == vertices_.end() || !vertices_.count(dest))
    {
        cerr << "error: cannot remove edge: source or destination vertex doesn't exist" << endl;
        return false;
    }

    int success = src_it->second->removeEdge(dest);
    if (!success)
    {
        cerr << "error: cannot remove edge: edge doesn't exists" << endl;
    }

    return success;
}

// Time complexity: O(ElogV)
template <class T>
const vector<T> Graph<T>::bestPath(const T src, const T dest) const
{
    if (!vertices_.count(src) || !vertices_.count(dest))
    {
        throw std::invalid_argument("cannot find best path because source and/or destination vertices don't exist");
    }

    /* Apply Dijkstra's shortest path algorithm to the graph */

    map<const T, double> dist;        // Maps a vertex to its distance from src
    map<const T, T> previous_vertex;  // Maps a vertex to the previous vertex along the best path
    map<const T, T> previous_edge;    // Maps a vertex to the incoming edge along the best path

    for (auto const &pair : vertices_)
    {
        dist[pair.first] = std::numeric_limits<double>::infinity();
    }

    dist[src] = 0.0f;

    // Initialize priority queue with source vertex
    priority_queue<pair<double, T>, vector<pair<double, T> >, greater<pair<double, T> > > vertices_queue;
    vertices_queue.push(pair<double, T>(0, src));

    while (!vertices_queue.empty())
    {
        // Get the vertex with the smallest distance from src
        pair<double, T> smallest_pair = vertices_queue.top();
        double dist_to_vertex = smallest_pair.first;
        const T vertex = smallest_pair.second; 
        vertices_queue.pop();

        // Stop searching if reached the dest vertex
        if (vertex == dest)
            break;

        // Check if there are now shorter distances to each neighbour
        const vector<Edge<T>* > edge_list = vertices_.find(vertex)->second->edgeList();
        for (const Edge<T>* edge : edge_list)
        {
            double new_dist = dist_to_vertex + edge->weight();
            const T edge_dest = edge->dest();
            if (new_dist < dist[edge_dest])
            {
                // Note: We may push duplicate vertices to the priority queue but this is expected
                vertices_queue.push(pair<double, T>(new_dist, edge_dest));

                dist[edge_dest] = new_dist;
                previous_vertex[edge_dest] = vertex;
                if (use_edge_id_)
                    previous_edge[edge_dest] = edge->id();
            }
        }
    }

    // Return an empty vector if dest vertex is not reachable from src vertex
    if (dist[dest] == std::numeric_limits<double>::infinity())
        return vector<T>();

    /* Get the best path */

    vector<T> path;
    T vertex_id = dest;
    if (use_edge_id_)
    {
        // Populate path with edge IDs along best path
        while (vertex_id != src)
        {
            path.push_back(previous_edge[vertex_id]);
            vertex_id = previous_vertex[vertex_id];
        }
    }
    else
    {
        // Populate path vith vertex IDs along best path
        path.push_back(dest);
        while (vertex_id != src)
        {
            path.push_back(previous_vertex[vertex_id]);
            vertex_id = previous_vertex[vertex_id];
        }
    }

    std::reverse(path.begin(), path.end());

    return path;
}

template <class T>
void Graph<T>::print() const
{
    cout << "Graph:" << endl;
    for (auto const &pair : vertices_)
    {
        cout << pair.first << ": ";
        const vector<Edge<T>* > edges = pair.second->edgeList();
        for (class vector<Edge<T>* >::const_iterator it = edges.begin(); it != edges.end(); it++)
        {
            cout << "[" << (*it)->dest() << ":" << (*it)->weight() << "]";
            if (it != edges.end() - 1)
            {
                cout << ", ";
            }
        }
        cout << endl;
    }
}

#endif  // GRAPH_H