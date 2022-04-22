{% from 'badger-lib.sls' import enable_service with context %}
{{ enable_service('toetic', 'amcl') }}
