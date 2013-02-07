module Transformer
    # Module that extends the TaskContext class itself
    module TaskContextModelExtension
        def each_transform_output
            return enum_for(:each_transform_output) if !block_given?
            return if !(tr = transformer)

            each_output_port do |port|
                if associated_transform = tr.find_transform_of_port(port.name)
                    yield(port, associated_transform.from, associated_transform.to)
                end
            end
        end
    end
end

