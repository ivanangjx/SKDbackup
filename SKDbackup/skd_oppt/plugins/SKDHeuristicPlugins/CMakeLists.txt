
# Kamikaze Heuristics
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicPlugin PRIVATE ${PROJECT_SOURCE_DIR})

# Kamikaze Heuristics New - straightline to optimal point then x axis
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicNewPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicNewPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicNewPlugin PRIVATE ${PROJECT_SOURCE_DIR})

# Kamikaze Heuristics Direct (With init max dist) - straightline to optimal point then x axis
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicDirectPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicDirectPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicDirectPlugin PRIVATE ${PROJECT_SOURCE_DIR})


# Kamikaze Heuristics Direct RETURNS PAIR (With init max dist) - straightline to optimal point then x axis
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicDirectPairPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicDirectPairPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicDirectPairPlugin PRIVATE ${PROJECT_SOURCE_DIR})




# Kamikaze Heuristics XY - Y axis, then X axis, check violation at each step
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicXYPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicXYPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicXYPlugin PRIVATE ${PROJECT_SOURCE_DIR})





# Safety Heuristics
ADD_HEURISTIC_PLUGIN(SafeTrajGenHeuristicPlugin ${CMAKE_CURRENT_LIST_DIR}/SafeTrajGenHeuristicPlugin.cpp)
target_include_directories(SafeTrajGenHeuristicPlugin PRIVATE ${PROJECT_SOURCE_DIR})

# Kamikaze Constraint Heuristics
ADD_HEURISTIC_PLUGIN(KamikazeTrajGenHeuristicConstraintPlugin ${CMAKE_CURRENT_LIST_DIR}/KamikazeTrajGenHeuristicConstraintPlugin.cpp)
target_include_directories(KamikazeTrajGenHeuristicConstraintPlugin PRIVATE ${PROJECT_SOURCE_DIR})

