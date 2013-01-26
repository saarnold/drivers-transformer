module Transformer
    # Extension to syskit profiles
    #
    # It gives access to the transformer configuration
    module ProfileExtension
        def transformer
            @transformer ||= Configuration.new
            if block_given?
                @transformer.instance_eval(&proc)
            end
            @transformer
        end

        # Called by Profile to modify an instance requirement object w.r.t. the
        # transformer information 
        def inject_di_context(req)
            req.transformer.merge(transformer)
        end
    end
end
