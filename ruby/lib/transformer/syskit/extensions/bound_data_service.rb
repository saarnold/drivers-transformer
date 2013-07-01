module Transformer
    module BoundDataServiceExtension
        include ConcreteComponentExtension

        def each_transform_output
            mappings = model.port_mappings_for_task
            component.each_transform_output do |port, port_from, port_to|
                if mapped = mappings.find { |_, task_port| port.name == task_port }
                    yield(find_output_port(mapped.first), port_from, port_to)
                end
            end
        end

        # Given a port associated with a transformer transformation, assign the
        # given frames to this local transformation
        def select_port_for_transform(port, from, to)
            if port.respond_to?(:name)
                port = port.name
            end

            task_port_name = model.port_mappings_for_task[port]
            if !task_port_name
                raise ArgumentError, "#{port} is not a known output port of #{self}"
            end

            task_port = component.find_output_port(task_port_name)
            if !task_port
                raise ArgumentError, "#{port} is not an output port of #{self}"
            end
            component.select_port_for_transform(task_port, from, to)
        end
    end
end
