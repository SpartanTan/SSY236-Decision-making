:- register_ros_package(knowrob_maps).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_common).


:- consult('instance_utils').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://barbot_brain/owl/group2.owl').
%:- rdf_db:rdf_register_ns(drinkOntology, 'http://www.semanticweb.org/tzc/ontologies/2022/10/drink-onotology#', [keep(true)]).
:- rdf_db:rdf_register_ns(drinkOntology, 'http://www.semanticweb.org/vamsi/ontologies/2023/0/group2-ontology-3#', [keep(true)]).

