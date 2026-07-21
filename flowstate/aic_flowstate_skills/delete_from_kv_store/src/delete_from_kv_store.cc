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

#include "delete_from_kv_store.h"

#include <memory>
#include <string>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/wrappers.pb.h"
#include "intrinsic/platform/pubsub/pubsub.h"
#include "delete_from_kv_store.pb.h"

std::unique_ptr<intrinsic::skills::SkillInterface> DeleteFromKVStore::CreateSkill() {
  return std::make_unique<DeleteFromKVStore>();
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> DeleteFromKVStore::Preview(
    const intrinsic::skills::PreviewRequest& /*request*/,
    intrinsic::skills::PreviewContext& /*context*/) {
  return absl::UnimplementedError("Preview not supported for this skill");
}

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> DeleteFromKVStore::Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& /*context*/) {
  INTR_ASSIGN_OR_RETURN(
      auto params,
      request.params<ai::flowstate::DeleteFromKVStoreParams>());

  std::string key = params.key();
  if (key.empty()) {
    LOG(ERROR) << "DeleteFromKVStore failed: Storage location key is empty.";
    return absl::InvalidArgumentError("Storage location key must not be empty");
  }

  LOG(INFO) << "Executing DeleteFromKVStore for key: '" << key << "'";

  // Connect to default PubSub KVStore ("kv_store" prefix)
  intrinsic::PubSub pubsub;
  INTR_ASSIGN_OR_RETURN(intrinsic::KeyValueStore kvstore,
                        pubsub.KeyValueStore());

  LOG(INFO) << "Deleting key '" << key << "' from KV store";

  // Delete key from KV store
  absl::Status status = kvstore.Delete(key);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to delete key '" << key << "' from KV store: " << status.message();
    return status;
  }

  LOG(INFO) << "Successfully deleted key '" << key << "' from KV store";

  auto result = std::make_unique<ai::flowstate::DeleteFromKVStoreResult>();
  result->set_success(true);
  result->set_message(absl::StrCat("Successfully deleted key '", key, "' from KV store"));

  return result;
}
