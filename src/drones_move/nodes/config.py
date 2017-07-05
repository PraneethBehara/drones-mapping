
turtlebot = dict(
    pub = dict(
        odom = '/odom'
    )
)

rtabmap = dict(
    pub = dict(
        grid_map = '/map'
    ,   proj_map = '/rtabmap/proj_map'
    )
)

move_base = dict(
    pub = dict(
        status = '/move_base/status'
    )
,   srv  = dict(
        make_plan = '/move_base/make_plan'
    )
)

map_combiner = dict(
    name = 'map_combiner'
,   pub  = dict(
        map_explore = 'map_explore'
    )
)

bot_mover = dict(
    name = 'bot_mover'
,   pub  = dict(
        cmd_vel     = '/cmd_vel_mux/input/navi'
    ,   move_goal   = '/move_base_simple/goal'
    ,   ready_state = '/ready_state'
    )
,   srv  = dict(
        move_bot = 'move_bot'
    )
)

controller = dict(
    name = 'controller'
,   pub  = dict(
        viz_goal = 'viz_goal'
    ,   viz_inflated_obstacles = 'viz_inflated_obstacles'
    ,   viz_filtered_horizon = 'viz_filtered_horizon'
    )
)

costmaps = dict(
	name = 'costmaps'
,	pub = dict(
  		costmap = '/move_base/local_costmap/costmap'
  	)
)