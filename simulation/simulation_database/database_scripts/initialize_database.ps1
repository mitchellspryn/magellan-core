$env:PGPASSWORD = ''
$username = 'postgres'

psql -U $username -c 'DROP DATABASE IF EXISTS simulation'
psql -U $username -c 'CREATE DATABASE simulation'

psql -U $username -d simulation -f "table_and_type_definitions.sql"

psql -U $username -d simulation -f "vector_math_functions.sql"
psql -U $username -d simulation -f "complete_simulation_run_function.sql"
psql -U $username -d simulation -f "compute_run_score_function.sql"
psql -U $username -d simulation -f "create_simulation_run_function.sql"
psql -U $username -d simulation -f "delete_simulation_run_function.sql"
psql -U $username -d simulation -f "mark_bonus_cone_visited.sql"
