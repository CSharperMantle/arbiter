# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: SerialHostMessage.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import ClientInfo_pb2 as ClientInfo__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x17SerialHostMessage.proto\x1a\x10\x43lientInfo.proto\"/\n\x0b\x43lientsList\x12 \n\x0b\x63lient_info\x18\x01 \x03(\x0b\x32\x0b.ClientInfo\"\xd8\x01\n\x11SerialHostMessage\x12%\n\x04type\x18\x01 \x01(\x0e\x32\x17.SerialHostMessage.Type\x12\x1f\n\x07\x63lients\x18\x02 \x01(\x0b\x32\x0c.ClientsListH\x00\x12\"\n\x0b\x61rbitration\x18\x03 \x01(\x0b\x32\x0b.ClientInfoH\x00\"L\n\x04Type\x12\x16\n\x12QUERY_CONFIRMATION\x10\x00\x12\x16\n\x12RESET_CONFIRMATION\x10\x01\x12\x14\n\x10\x41RBITRATION_DONE\x10\x02\x42\t\n\x07\x63ontentb\x06proto3')



_CLIENTSLIST = DESCRIPTOR.message_types_by_name['ClientsList']
_SERIALHOSTMESSAGE = DESCRIPTOR.message_types_by_name['SerialHostMessage']
_SERIALHOSTMESSAGE_TYPE = _SERIALHOSTMESSAGE.enum_types_by_name['Type']
ClientsList = _reflection.GeneratedProtocolMessageType('ClientsList', (_message.Message,), {
  'DESCRIPTOR' : _CLIENTSLIST,
  '__module__' : 'SerialHostMessage_pb2'
  # @@protoc_insertion_point(class_scope:ClientsList)
  })
_sym_db.RegisterMessage(ClientsList)

SerialHostMessage = _reflection.GeneratedProtocolMessageType('SerialHostMessage', (_message.Message,), {
  'DESCRIPTOR' : _SERIALHOSTMESSAGE,
  '__module__' : 'SerialHostMessage_pb2'
  # @@protoc_insertion_point(class_scope:SerialHostMessage)
  })
_sym_db.RegisterMessage(SerialHostMessage)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _CLIENTSLIST._serialized_start=45
  _CLIENTSLIST._serialized_end=92
  _SERIALHOSTMESSAGE._serialized_start=95
  _SERIALHOSTMESSAGE._serialized_end=311
  _SERIALHOSTMESSAGE_TYPE._serialized_start=224
  _SERIALHOSTMESSAGE_TYPE._serialized_end=300
# @@protoc_insertion_point(module_scope)