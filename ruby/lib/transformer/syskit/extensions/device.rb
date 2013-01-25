module Transformer
    # Module used to extend the device specification objects with the ability
    # to specify frames
    #
    # The #frame attribute allows to specify in which frame this device
    # produces information.
    module DeviceExtension
        ## 
        # Declares the frame in which this device produces data
        dsl_attribute :frame do |value|
            value.to_str
        end

        ## 
        # Declares the frame transformation in which this device produces
        # transformations
        dsl_attribute :frame_transform do |value|
            if value.kind_of?(Transform)
                value
            else
                if !value.kind_of?(Hash)
                    raise ArgumentError, "expected a from => to mapping, got #{value}"
                elsif value.size > 1
                    raise ArgumentError, "more than one transformation provided"
                end
                Transform.new(*value.to_a.first)
            end
        end
    end
end

