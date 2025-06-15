import chromadb
from openai import OpenAI
import rospy


# 大模型初始化
def init_embedding_model(api_key: str, base_url: str):
    client = OpenAI(
        api_key  = api_key,  # 如果您没有配置环境变量，请在此处用您的API Key进行替换
        base_url = base_url  # 百炼服务的base_url
    )
    return client

# Embedding
def embedding(client, text_to_embedding: str):
    """
        执行文本的embedding操作
    :param client: embedding模型实例化对象
    :param text_to_embedding:  需要进行向量化的文本
    :return:       返回向量化后的数据
    """
    completion = client.embeddings.create(
        model= rospy.get_param('~embedding/model'),
        input= text_to_embedding,
        dimensions=1024,
        encoding_format="float"
    )
    return completion.data[0].embedding


# 实例化一个数据库保存向量数据和保存向量对象的集合（类似于表）
def init_chroma_db(table_name: str, save_path: str = None):
    if save_path is None:
        save_path = rospy.get_param('~RAG_path')
    chromadb_client = chromadb.PersistentClient(save_path)
    chromadb_collection = chromadb_client.get_or_create_collection(table_name)
    return chromadb_collection


# 查询函数
def search_date(query: str):
    # Embdding模型初始化
    embedding_client = init_embedding_model(rospy.get_param('~embedding/api_key'), rospy.get_param('~embedding/base_url'))

    # 生成向量数据库实例并保存在子文件下
    chromadb_collection = init_chroma_db("LLM_Robot_Memory")

    query_vector = embedding(embedding_client, query)
    results = chromadb_collection.query(
        query_embeddings=[query_vector],
        n_results=2
    )

    return results['documents'][0]
























