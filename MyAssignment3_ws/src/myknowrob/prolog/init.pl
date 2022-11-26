:- register_ros_package(knowrob_maps).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowrob_common).


:- consult('instance_utils').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://knowrob_tutorial/owl/ssy236Ontology.owl').
:- rdf_db:rdf_register_ns(ssy236Ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).


