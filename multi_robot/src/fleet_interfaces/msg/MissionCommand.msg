std_msgs/Header header
string session_id
string issuer_id
string command_id
uint64 revision
string kind
string argument
# Fixed MEMBERS order: robot1, robot2, robot3. Empty means not addressed.
string[] participant_boot_ids
# Same-host monotonic proposal deadline; a live committed quorum may relay later.
float64 expires_at
