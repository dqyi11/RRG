#ifndef RRG_H
#define RRG_H

#include <vector>
#include <list>

#include "KDTree2D.h"

typedef double (*COST_FUNC_PTR)(POS2D, POS2D, double**, void*);

class RRGNode {

public:
    RRGNode( POS2D pos );

    bool operator==( const RRGNode &other );

    double   m_cost;
    RRGNode* mp_parent;
    POS2D    m_pos;
    std::list<RRGNode*> m_child_nodes;
};

class Path {

public:
    Path(POS2D start, POS2D goal);
    ~Path();

    double m_cost;
    POS2D  m_start;
    POS2D  m_goal;
    std::vector<POS2D> m_way_points;
};

class RRG {

public:
    RRG(int width, int height, int segment_length);
    ~RRG();

    RRGNode* init( POS2D start, POS2D goal, COST_FUNC_PTR p_func, double** pp_cost_distrinution );

    void load_map( int** pp_map );

    int get_sampling_width() { return _sampling_width; }
    int get_sampling_height() { return _sampling_height; }
    int get_current_iteration() { return _current_iteration; }

    std::list<RRGNode*>& get_nodes() { return _nodes; }

    int**& get_map_info() { return _pp_map_info; }
    double get_ball_radius() { return _ball_radius; }

    void extend();
    Path* find_path();

    void dump_distribution(std::string filename);

protected:
    POS2D _sampling();
    POS2D _steer( POS2D pos_a, POS2D pos_b );

    KDNode2D _find_nearest( POS2D pos );
    std::list<KDNode2D> _find_near( POS2D pos );

    bool _is_obstacle_free( POS2D pos_a, POS2D pos_b );
    bool _is_in_obstacle( POS2D pos );
    bool _contains( POS2D pos );

    double _calculate_cost( POS2D& pos_a, POS2D& pos_b );

    RRGNode* _create_new_node( POS2D pos );
    bool _remove_edge( RRGNode* p_node_parent, RRGNode* p_node_child );
    bool _has_edge( RRGNode* p_node_parent, RRGNode* p_node_child );
    bool _add_edge( RRGNode* p_node_parent, RRGNode* p_node_child );

    std::list<RRGNode*> _find_all_children( RRGNode* node );

    void _attach_new_node( RRGNode* p_node_new, RRGNode* p_nearest_node, std::list<RRGNode*> near_nodes );
    void _rewire_near_nodes( RRGNode* p_node_new, std::list<RRGNode*> near_nodes );
    void _update_cost_to_children( RRGNode* p_node, double delta_cost );
    bool _get_closet_to_goal( RRGNode*& p_node_closet_to_goal, double& delta_cost );

    RRGNode* _find_ancestor( RRGNode* p_node );

private:
    POS2D    _start;
    POS2D    _goal;
    RRGNode* _p_root;

    int _sampling_width;
    int _sampling_height;

    int** _pp_map_info;

    KDTree2D*     _p_kd_tree;
    COST_FUNC_PTR _p_cost_func;
    double**      _pp_cost_distribution;

    std::list<RRGNode*> _nodes;

    double _range;
    double _ball_radius;
    double _segment_length;
    int    _obs_check_resolution;

    double _theta;
    int    _current_iteration;
};

inline RRGNode* get_ancestor( RRGNode * node ) {
    if( NULL == node ) {
        return NULL;
    }
    if( NULL == node->mp_parent ) {
        return node;
    }
    else {
        return get_ancestor( node->mp_parent );
    }
}

inline void get_parent_node_list( RRGNode * node, std::list<RRGNode*>& path ) {
    if( node==NULL ) {
        return;
    }
    path.push_back( node );
    get_parent_node_list( node->mp_parent, path );
    return;
}

#endif // RRG_H
