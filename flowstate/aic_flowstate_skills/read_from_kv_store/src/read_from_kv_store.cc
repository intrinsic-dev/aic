/*
 * Copyright (C) 2026 Intrinsic Innovation LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "read_from_kv_store.h"

#include <memory>
#include <string>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/wrappers.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "read_from_kv_store.pb.h"

std::unique_ptr<intrinsic::skills::SkillInterface>
ReadFromKVStore::CreateSkill() {
  return std::make_unique<ReadFromKVStore>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
ReadFromKVStore::Preview(const intrinsic::skills::PreviewRequest& /*request*/,
                         intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Preview not supported for this skill");
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
ReadFromKVStore::Execute(const intrinsic::skills::ExecuteRequest& request,
                         intrinsic::skills::ExecuteContext& /*context*/) {
  INTR_ASSIGN_OR_RETURN(auto params,
                        request.params<ai::flowstate::ReadFromKVStoreParams>());

  std::string key = params.key();
  if (key.empty()) {
    LOG(ERROR) << "ReadFromKVStore failed: Storage location key is empty.";
    return absl::InvalidArgumentError("Storage location key must not be empty");
  }

  LOG(INFO) << "Executing ReadFromKVStore for key: '" << key << "'";

  // Connect to default PubSub KVStore ("kv_store" prefix) using pre-connected
  // member pubsub_
  INTR_ASSIGN_OR_RETURN(intrinsic::KeyValueStore kvstore,
                        pubsub_.KeyValueStore());

  LOG(INFO) << "Retrieving value for key '" << key << "' from KV store";

  // Retrieve the Int64Value directly from KV store
  auto value_or = kvstore.Get<google::protobuf::Int64Value>(key);
  if (!value_or.ok()) {
    LOG(ERROR) << "Failed to retrieve key '" << key
               << "' from KV store: " << value_or.status().message();
    return value_or.status();
  }

  google::protobuf::Int64Value value_msg = value_or.value();
  LOG(INFO) << "Successfully retrieved counter value " << value_msg.value()
            << " from key '" << key << "'";

  auto result = std::make_unique<ai::flowstate::ReadFromKVStoreResult>();
  result->set_count(value_msg.value());
  result->set_success(true);
  result->set_message(absl::StrCat("Successfully retrieved counter value ",
                                   value_msg.value(), " from key '", key, "'"));

  return result;
}
