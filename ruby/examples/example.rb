frames :laser, :body, :head, :tilt_head, :odometry    

dynamic_transform(:laser, :tilt_head, 'producer')
dynamic_transform(:body, :odometry, 'producer')
static_transform(:tilt_head, :head, Eigen::Vector3.new(0, 1, 0))
static_transform(:head, :body, Eigen::Quaternion.Identity)

