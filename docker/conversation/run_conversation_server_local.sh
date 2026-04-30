#!/usr/bin/env bash

set -euo pipefail

if [[ -f /opt/cabot/.env ]]; then
    set -a
    source /opt/cabot/.env
    set +a
fi

if [[ -n "${HOST_TZ:-}" ]]; then
    export TZ="${HOST_TZ}"
elif [[ -f /etc/timezone ]]; then
    export TZ="$(tr -d '[:space:]' < /etc/timezone)"
fi

log_dir="${CABOT_CONVERSATION_LOG_DIR:-/opt/cabot/log}"
tools_dir="${CABOT_CONVERSATION_TOOLS_DIR:-/opt/cabot/tools}"
summary_interval="${CABOT_CONVERSATION_SUMMARY_INTERVAL_SECONDS:-5}"
timestamp="$(date +%Y-%m-%d-%H-%M-%S)"
base_name="cabot_convasation_${timestamp}"
raw_log="${log_dir}/${base_name}.log"
format_json="${log_dir}/${base_name}.format.json"
summary_log="${log_dir}/${base_name}.summary.log"
latest_raw="${log_dir}/convasation.log"
latest_format="${log_dir}/format.json"
latest_summary="${log_dir}/summary.log"

mkdir -p "${log_dir}"
touch "${raw_log}" "${format_json}" "${summary_log}"
ln -sfn "$(basename "${raw_log}")" "${latest_raw}"
ln -sfn "$(basename "${format_json}")" "${latest_format}"
ln -sfn "$(basename "${summary_log}")" "${latest_summary}"

update_summary() {
    if [[ ! -s "${raw_log}" ]] || [[ ! -f "${tools_dir}/decode_console_logs.py" ]]; then
        return 0
    fi

    local tmp_summary="${summary_log}.tmp"
    local tmp_format="${format_json}.tmp"

    python3 "${tools_dir}/decode_console_logs.py" "${raw_log}" "${tmp_format}" > "${tmp_summary}" 2>/dev/null || return 0
    mv "${tmp_format}" "${format_json}"
    mv "${tmp_summary}" "${summary_log}"
}

cleanup() {
    local exit_code=$?

    if [[ -n "${summary_pid:-}" ]]; then
        kill "${summary_pid}" 2>/dev/null || true
        wait "${summary_pid}" 2>/dev/null || true
    fi

    update_summary
    exit "${exit_code}"
}

trap cleanup EXIT INT TERM

(
    while true; do
        sleep "${summary_interval}"
        update_summary
    done
) &
summary_pid=$!

echo "conversation log: ${raw_log}" | tee -a "${raw_log}"
echo "conversation format: ${format_json}" | tee -a "${raw_log}"
echo "conversation summary: ${summary_log}" | tee -a "${raw_log}"

uv run uvicorn app.api.main:app --host 0.0.0.0 --port "${PORT:-8000}" --log-level debug 2>&1 | tee -a "${raw_log}"
exit "${PIPESTATUS[0]}"
