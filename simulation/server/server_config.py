import json

class ServerConfig(object):
    def __init__(self, file_path):
        with open(file_path, 'r') as f:
            config_str = f.read()
        config_obj = json.loads(config_str)

        required_notnull_properties = [
            'db_database',
            'db_host',
            'db_password',
            'db_port',
            'db_user',
            'simulator_path'
        ]

        for required_notnull_property in required_notnull_properties:
            self.__throw_if_property_not_exist(required_notnull_property, allow_null=False)

    def __throw_if_property_not_exist(self, property_name, allow_null = False):
       if (not hasattr(self, property_name)):
           raise ValueError('Required property {0} is missing from server configuration.'.format(property_name))

       if (not allow_null and getattr(self, property_name) is None):
           raise ValueError('Required property {0} is null, but has been marked not-null.'.format(property_name))



