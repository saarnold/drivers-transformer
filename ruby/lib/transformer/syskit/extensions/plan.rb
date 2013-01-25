module Transformer
    module PlanExtension
        attribute(:transformer_configuration_state) do
            [Time.now, Types::Transformer::ConfigurationState.new]
        end
    end
end


