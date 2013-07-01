module Transformer
    module ConfigurationExtension
        attr_predicate :transformer_enabled?, true

        def transformer_enabled=(value)
            if value
                Transformer::SyskitPlugin.enable
            end
            @transformer_enabled = value
        end
    end
end

