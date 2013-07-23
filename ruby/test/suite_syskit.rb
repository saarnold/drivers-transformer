if !ENV['SYSKIT_ENABLE_COVERAGE']
    ENV['SYSKIT_ENABLE_COVERAGE'] = '2'
end
require './test/syskit/test_composition_extension'
require './test/syskit/test_syskit_plugin'
require './test/syskit/test_task_context_extension'
require './test/syskit/test_transformer'

Transformer.logger = Logger.new(File.open("/dev/null", 'w'))
Transformer.logger.level = Logger::DEBUG
Syskit.logger = Logger.new(File.open("/dev/null", 'w'))

