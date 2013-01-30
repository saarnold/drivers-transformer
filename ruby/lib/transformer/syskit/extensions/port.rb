module Transformer
    # Extension to Syskit::Port
    module PortExtension
        # Returns a transformation object that represents the transformation
        # this port is producing
        #
        # @return [Transformation,nil] the generated transformation, or nil if
        #   this port does not produce any. The transformation returned includes
        #   global frame names, not task-local ones.
        def produced_transformation
            component.find_transform_of_port(self)
        end
    end
end
