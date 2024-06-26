const char cdi[] PROGMEM = { "\
<?xml version='1.0'?> \
<cdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='http://openlcb.org/schema/cdi/1/1/cdi.xsd'> \
<identification> \
<manufacturer>Snowball Creek</manufacturer> \
<model>Crossing Gate Controller</model> \
<hardwareVersion>1.0</hardwareVersion> \
<softwareVersion>0.1</softwareVersion> \
</identification> \
<acdi/> \
<segment space='251'> \
<name>Node ID</name> \
<group> \
<name>Your name and description for this node</name> \
<string size='63'> \
<name>Node Name</name> \
</string> \
<string size='64' offset='1'> \
<name>Node Description</name> \
</string> \
</group> \
</segment> \
<segment space='253'> \
<name>Routes</name> \
<group replication='8'> \
<name>Route</name> \
<repname>Route</repname> \
<group replication='4'> \
<name>Sensor Inputs</name> \
<repname>Sensor Inputs</repname> \
<int size='1'> \
<name>GPIO Number</name> \
<description>GPIO number to use as input to this sensor on this route.  Set to 0 to disable and use EventIDs.</description> \
</int> \
<int size='1'> \
<name>GPIO Type</name> \
<description>GPIO Type(analog or normal)</description> \
<map> \
<relation> \
<property>0</property> \
<value>GPIO</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Analog GPIO</value> \
</relation> \
</map> \
</int> \
<int size='1'> \
<name>Polarity</name> \
<description>GPIO polarity</description> \
<map> \
<relation> \
<property>0</property> \
<value>High</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Low</value> \
</relation> \
</map> \
</int> \
<!-- 1 byte padding here --> \
<int size='2' offset='1'> \
<name>Analog Value</name> \
<description>If using an analog pin, values above this will be 'on'</description> \
<min>0</min> \
<max>1023</max> \
<default>250</default> \
</int> \
<eventid> \
<name>Event Id ON</name> \
<description>If not using GPIO, event ID to indicate that this sensor is on</description> \
</eventid> \
<eventid> \
<name>Event Id OFF</name> \
<description>If not using GPIO, event ID to indicate that this sensor is off</description> \
</eventid> \
<int size='2'> \
<name>Debounce ON time</name> \
<description>The time(in milliseconds) for a debounce delay when activated</description> \
</int> \
<int size='2'> \
<name>Debounce OFF time</name> \
<description>The time(in milliseconds) for a debounce delay when deactivated</description> \
</int> \
<group offset='38'/> \
</group> \
<!-- Switch inputs --> \
<group replication='8'> \
<name>Switch Inputs</name> \
<repname>Switch Input</repname> \
<int size='1'> \
<name>GPIO Number</name> \
<description>GPIO number to use as input to this sensor on this route.  Set to 0 to disable and use EventIDs.</description> \
</int> \
<int size='1'> \
<name>Polarity</name> \
<description>GPIO polarity</description> \
<map> \
<relation> \
<property>0</property> \
<value>High</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Low</value> \
</relation> \
</map> \
</int> \
<int size='1'> \
<name>Route Posistion</name> \
<description>The posistion this switch needs to be in for this route to be valid</description> \
<map> \
<relation> \
<property>0</property> \
<value>Normal</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Reverse</value> \
</relation> \
</map> \
</int> \
<eventid> \
<name>Event Id Normal</name> \
<description>If not using GPIO, event ID to indicate that this switch is normal</description> \
</eventid> \
<eventid> \
<name>Event Id Reverse</name> \
<description>If not using GPIO, event ID to indicate that this switch is reversed</description> \
</eventid> \
<group offset='45'/> \
</group> \
</group> \
</segment> \
<segment space='253' origin='8192'> \
<name>General Config</name> \
<!-- invisible EEPROM version here --> \
<int size='1' offset='1'> \
<name>Timeout</name> \
<description>Timeout value for system to reset and become 'inactive'.  This value is in seconds</description> \
<min>0</min> \
<max>120</max> \
<default>25</default> \
</int> \
<int size='1'> \
<name>Gate output</name> \
<description>GPIO to turn ON when gates should be down</description> \
<min>0</min> \
<max>100</max> \
</int> \
<int size='1'> \
<name>Gate output 2</name> \
<description>GPIO to turn ON when gates should be down</description> \
<min>0</min> \
<max>100</max> \
</int> \
<int size='1'> \
<name>LED GPIO 1</name> \
<description>GPIO to turn ON to toggle LED 1 on the crossing</description> \
<min>0</min> \
<max>100</max> \
</int> \
<int size='1'> \
<name>LED GPIO 2</name> \
<description>GPIO to turn ON to toggle LED 2 on the crossing</description> \
<min>0</min> \
<max>100</max> \
</int> \
<int size='2'> \
<name>LED Flash Time</name> \
<description>How long for each LED on the crossing to flash in milliseconds</description> \
<min>0</min> \
<max>1000</max> \
<default>250</default> \
</int> \
<int size='1'> \
<name>Bell GPIO</name> \
<description>The GPIO used to ring the bell</description> \
<min>0</min> \
<max>128</max> \
</int> \
<int size='1'> \
<name>Bell Behavior</name> \
<description>The behavior of the bell</description> \
<map> \
<relation> \
<property>0</property> \
<value>No ring</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Ring while gates lowering</value> \
</relation> \
<relation> \
<property>2</property> \
<value>Ring while gates raising</value> \
</relation> \
<relation> \
<property>3</property> \
<value>Ring while gates lowering and raising</value> \
</relation> \
<relation> \
<property>4</property> \
<value>Ring while flashing</value> \
</relation> \
</map> \
</int> \
<int size='2'> \
<name>Bell ring time</name> \
<description>Bell ring time(in milliseconds) when gates are raising/lowering</description> \
<min>0</min> \
<max>10000</max> \
</int> \
</segment> \
<segment space='250'> \
<name>Debug values(Raw GPIO)</name> \
<group replication='8'> \
<name>Route input values</name> \
<repname>Route </repname> \
<group replication='4'> \
<name>Input Values</name> \
<repname>Input Value</repname> \
<int size='2'> \
<name>Input</name> \
<description>Raw value of GPIO</description> \
</int> \
</group> \
</group> \
</segment> \
</cdi> \
"};
