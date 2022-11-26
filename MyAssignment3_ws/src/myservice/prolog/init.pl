:- register_ros_package(knowrob_maps).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_common).


:- consult('instance_utils').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://myservice/owl/drink.owl').
:- rdf_db:rdf_register_ns(drinkOntology, 'http://www.semanticweb.org/tzc/ontologies/2022/10/drink-onotology#', [keep(true)]).


