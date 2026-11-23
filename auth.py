"""简洁的本地身份认证：PBKDF2 密码哈希与 HMAC 签名访问令牌。"""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import secrets
import time

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from sqlalchemy.orm import Session

import models
from database import SessionLocal, get_db


PASSWORD_ITERATIONS = 210_000
TOKEN_LIFETIME_SECONDS = 12 * 60 * 60
TOKEN_SECRET = os.getenv("UAV_AUTH_SECRET", "uav-local-development-secret-change-me")
DEFAULT_ADMIN_USERNAME = os.getenv("UAV_ADMIN_USERNAME", "test")
DEFAULT_ADMIN_PASSWORD = os.getenv("UAV_ADMIN_PASSWORD", "test")
DEFAULT_ADMIN_DISPLAY_NAME = os.getenv("UAV_ADMIN_DISPLAY_NAME", "测试用户")

bearer_scheme = HTTPBearer(auto_error=False)


def _base64url_encode(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")


def _base64url_decode(value: str) -> bytes:
    padding = "=" * (-len(value) % 4)
    return base64.urlsafe_b64decode(value + padding)


def hash_password(password: str) -> str:
    """使用随机盐和 PBKDF2-SHA256 保存密码，绝不存储明文。"""
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac(
        "sha256",
        password.encode("utf-8"),
        salt,
        PASSWORD_ITERATIONS,
    )
    return "$".join(
        (
            "pbkdf2_sha256",
            str(PASSWORD_ITERATIONS),
            _base64url_encode(salt),
            _base64url_encode(digest),
        )
    )


def verify_password(password: str, encoded_password: str) -> bool:
    try:
        algorithm, iterations_text, salt_text, digest_text = encoded_password.split("$", 3)
        if algorithm != "pbkdf2_sha256":
            return False
        expected = _base64url_decode(digest_text)
        actual = hashlib.pbkdf2_hmac(
            "sha256",
            password.encode("utf-8"),
            _base64url_decode(salt_text),
            int(iterations_text),
        )
        return hmac.compare_digest(actual, expected)
    except (TypeError, ValueError):
        return False


def create_access_token(user: models.User) -> tuple[str, int]:
    expires_at = int(time.time()) + TOKEN_LIFETIME_SECONDS
    payload = {
        "sub": user.id,
        "username": user.username,
        "exp": expires_at,
    }
    payload_text = _base64url_encode(
        json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    )
    signature = _base64url_encode(
        hmac.new(TOKEN_SECRET.encode("utf-8"), payload_text.encode("ascii"), hashlib.sha256).digest()
    )
    return f"{payload_text}.{signature}", TOKEN_LIFETIME_SECONDS


def decode_access_token(token: str) -> dict:
    try:
        payload_text, signature_text = token.split(".", 1)
        expected_signature = _base64url_encode(
            hmac.new(
                TOKEN_SECRET.encode("utf-8"),
                payload_text.encode("ascii"),
                hashlib.sha256,
            ).digest()
        )
        if not hmac.compare_digest(signature_text, expected_signature):
            raise ValueError("invalid signature")
        payload = json.loads(_base64url_decode(payload_text).decode("utf-8"))
        if int(payload["exp"]) <= int(time.time()):
            raise ValueError("expired token")
        return payload
    except (KeyError, TypeError, ValueError, OverflowError, json.JSONDecodeError) as exc:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="登录状态无效或已过期，请重新登录",
            headers={"WWW-Authenticate": "Bearer"},
        ) from exc


def authenticate_user(db: Session, username: str, password: str) -> models.User | None:
    user = db.query(models.User).filter(models.User.username == username).first()
    if not user or not user.active or not verify_password(password, user.password_hash):
        return None
    return user


def get_current_user(
    credentials: HTTPAuthorizationCredentials | None = Depends(bearer_scheme),
    db: Session = Depends(get_db),
) -> models.User:
    if credentials is None or credentials.scheme.lower() != "bearer":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="请先登录",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return get_user_from_token(credentials.credentials, db)


def get_user_from_token(token: str, db: Session) -> models.User:
    """Shared identity lookup for HTTP requests and first-message WebSocket auth."""
    payload = decode_access_token(token)
    try:
        user_id = int(payload["sub"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise HTTPException(status_code=401, detail="登录身份无效") from exc
    user = db.query(models.User).filter(models.User.id == user_id).first()
    if not user or not user.active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="用户不存在或已停用",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user


def ensure_default_admin() -> None:
    """首次启动时创建一个可登录的本地测试账号。"""
    db = SessionLocal()
    try:
        existing = db.query(models.User).filter(
            models.User.username == DEFAULT_ADMIN_USERNAME
        ).first()
        if existing:
            return
        db.add(
            models.User(
                username=DEFAULT_ADMIN_USERNAME,
                display_name=DEFAULT_ADMIN_DISPLAY_NAME,
                password_hash=hash_password(DEFAULT_ADMIN_PASSWORD),
                active=True,
            )
        )
        db.commit()
    finally:
        db.close()
