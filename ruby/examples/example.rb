
available_frames(
 :laser, :body, :head, :tilt_head, :odometry    
)

DynamicTransformation(:laser, :tilt_head, 'producer')
DynamicTransformation(:body, :odometry, 'producer')
#DynamicTransformation(:bla, :odometry, 'producer')
#DynamicTransformation(:bla, :blub, 'producer')
StaticTransformation(:tilt_head, :head)
StaticTransformation(:head, :body)

