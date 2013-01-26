module Transformer
    # Extension to syskit profiles
    #
    # It gives access to the transformer configuration
    module ProfileExtension
        # This is a normal transformer configuration with only a few added
        # tweaks
        class Configuration < Transformer::Configuration
            attr_reader :profile
            def initialize(profile)
                @profile = profile
                super()
            end

            def load(*path)
                args = path.dup
                args << Hash[:order => :specific_first]
                if !Roby.app.find_file(*args)
                    raise ArgumentError, "cannot find #{File.join(*path)}"
                end
                super(path)
            end

            def method_missing(m, *args, &block)
                if m.to_s =~ /_dev$/ || m.to_s =~ /_def$/
                    profile.send(m, *args, &block)
                else super
                end
            end
        end

        def transformer
            @transformer ||= Configuration.new(self)
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
